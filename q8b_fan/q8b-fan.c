#define _GNU_SOURCE
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdarg.h>
#include <math.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

/* Q8B fan controller V2: thermal PI / supervisor / manual fan calibration.
 * Uses the radxa_svc_glink ADSP hwmon interface.
 * No promise of a physically stationary fan when pwm1=0: ADSP can pulse it.
 */
#ifndef THERM_DIR
#define THERM_DIR "/sys/class/thermal"
#endif
#ifndef HWMON_DIR
#define HWMON_DIR "/sys/class/hwmon"
#endif
#ifndef LOCK_PATH
#define LOCK_PATH "/run/q8b-fan.lock"
#endif
#ifndef LOG_DIR
#define LOG_DIR "/var/log/q8b-fan"
#endif
#define MAX_ZONES 128
#define MAX_HISTORY 64
#define GUARD_TIMEOUT_MS 15000
#define CONFIG_DEFAULT "/etc/q8b-fan.conf"

enum group { OTHER, CPU, GPU, NPU, MEM, PMIC, VBAT, GROUPS };
static const char *group_names[GROUPS]={"other","cpu","gpu","npu","mem","pmic","vbat"};
enum state { OFF, ACTIVE, PROTECT, CAL };
static const char *state_names[]={"OFF","PI","PROTECT","CAL"};

struct config {
    double setpoint, off_hyst, off_confirm, sample_s, log_period, envelope_s, filter_tau;
    double kp, ki, pwm_up_rate, pwm_down_rate, kick_s;
    int pwm_start, pwm_hold, pwm_max;
    double protection_hyst, protection_recover_s;
    double limits[GROUPS];
};
static const struct config defaults={
    .setpoint=50,.off_hyst=3,.off_confirm=20,.sample_s=1,.log_period=5,
    .envelope_s=10,.filter_tau=8,.kp=12,.ki=0.10,
    .pwm_up_rate=30,.pwm_down_rate=8,.kick_s=1.2,
    .pwm_start=160,.pwm_hold=100,.pwm_max=255,
    .protection_hyst=5,.protection_recover_s=60,
    .limits={0,75,75,75,80,85,65}
};
static struct config cfg;
struct zone {char name[128], path[PATH_MAX], csv_name[132];enum group group;};
static struct zone zones[MAX_ZONES];
static int nzones;
static char mode_path[PATH_MAX], pwm_path[PATH_MAX], config_path[PATH_MAX];
static volatile sig_atomic_t terminate_requested, reload_requested;
static pid_t guardian_pid=-1;
static int guardian_fd=-1, lock_fd=-1;
static FILE *csv, *events;
static int record_enabled;
static char current_day[16],csv_path[PATH_MAX];
static double start_at, last_tick, last_log;
static int commanded_pwm=-1, mode_manual=0;
static int fan_owned=0;

static double now_mono(void) {struct timespec t;clock_gettime(CLOCK_MONOTONIC,&t);return t.tv_sec+t.tv_nsec*1e-9;}
static void handle_signal(int sig){if(sig==SIGHUP)reload_requested=1;else terminate_requested=1;}
static int join(char *dst,size_t n,const char *dir,const char *leaf){return snprintf(dst,n,"%s/%s",dir,leaf)<(int)n?0:-1;}
static int read_long(const char *path,long *v){
    FILE *f=fopen(path,"r");if(!f)return -1;
    char line[128];if(!fgets(line,sizeof(line),f)){fclose(f);return -1;}fclose(f);
    char *p;errno=0;long n=strtol(line,&p,10);
    if(errno || p==line)return -1;
    while(isspace((unsigned char)*p))p++;
    if(*p)return -1;
    *v=n;return 0;
}
static int read_text(const char *path,char *s,size_t size){
    FILE *f=fopen(path,"r");if(!f)return -1;
    if(!fgets(s,(int)size,f)){fclose(f);return -1;}
    fclose(f);s[strcspn(s,"\r\n")]='\0';return 0;
}
static int write_long(const char *path,long value){
    int fd=open(path,O_WRONLY|O_CLOEXEC);if(fd<0)return -1;
    char s[40];int n=snprintf(s,sizeof(s),"%ld\n",value);
    int ok=write(fd,s,(size_t)n)==n?0:-1;
    if(close(fd)<0)ok=-1;
    return ok;
}
static enum group classify(const char *n){
    if(strncmp(n,"cpu",3)==0 || strncmp(n,"cluster",7)==0)return CPU;
    if(strncmp(n,"gpuss",5)==0)return GPU;
    if(strncmp(n,"nsp",3)==0)return NPU;
    if(strncmp(n,"mem",3)==0)return MEM;
    if(strncmp(n,"pm",2)==0)return PMIC;
    if(strncmp(n,"vbat",4)==0)return VBAT;
    return OTHER;
}
static int cmp_zone(const void *a,const void *b){return strcmp(((const struct zone *)a)->name,((const struct zone *)b)->name);}
static int discover(void){
    DIR *d=opendir(THERM_DIR);if(!d){perror(THERM_DIR);return -1;}
    struct dirent *ent;
    while((ent=readdir(d))){
        if(strncmp(ent->d_name,"thermal_zone",12))continue;
        if(nzones>=MAX_ZONES){fprintf(stderr,"Too many thermal zones\n");closedir(d);return -1;}
        char dir[PATH_MAX],p[PATH_MAX],n[128];
        if(join(dir,sizeof(dir),THERM_DIR,ent->d_name) || join(p,sizeof(p),dir,"type") || read_text(p,n,sizeof(n)))continue;
        if(join(p,sizeof(p),dir,"temp") || access(p,R_OK))continue;
        struct zone *z=&zones[nzones++];
        snprintf(z->name,sizeof(z->name),"%s",n);
        snprintf(z->path,sizeof(z->path),"%s",p);
        z->group=classify(n);
        size_t j=0;
        for(size_t i=0;n[i] && j+1<sizeof(z->csv_name);i++){
            unsigned char c=(unsigned char)n[i];
            z->csv_name[j++]=isalnum(c)?(char)c:'_';
        }
        z->csv_name[j]=0;
    }
    closedir(d);
    qsort(zones,(size_t)nzones,sizeof(zones[0]),cmp_zone);
    int group_count[GROUPS]={0};
    for(int i=0;i<nzones;i++)group_count[zones[i].group]++;
    for(int g=CPU;g<GROUPS;g++){
        if(!group_count[g]){
            fprintf(stderr,"Missing required thermal group: %s\n",group_names[g]);
            return -1;
        }
    }
    d=opendir(HWMON_DIR);if(!d){perror(HWMON_DIR);return -1;}
    int matches=0;
    while((ent=readdir(d))){
        if(strncmp(ent->d_name,"hwmon",5))continue;
        char dir[PATH_MAX],path[PATH_MAX],name[128];
        if(join(dir,sizeof(dir),HWMON_DIR,ent->d_name)||join(path,sizeof(path),dir,"name")||read_text(path,name,sizeof(name)))continue;
        if(strcmp(name,"radxa_svc_glink"))continue;
        if(join(mode_path,sizeof(mode_path),dir,"pwm1_enable")||join(pwm_path,sizeof(pwm_path),dir,"pwm1"))continue;
        matches++;
    }
    closedir(d);
    long mode,pwm;
    if(matches!=1 || read_long(mode_path,&mode) || read_long(pwm_path,&pwm)){
        fprintf(stderr,"Expected one readable radxa_svc_glink, found %d\n",matches);return -1;
    }
    printf("Sensors=%d; radxa_svc_glink mode=%ld, PWM=%ld\n",nzones,mode,pwm);
    return 0;
}
static char *trim(char *s){
    while(isspace((unsigned char)*s))s++;
    size_t n=strlen(s);while(n>0 && isspace((unsigned char)s[n-1]))s[--n]=0;
    return s;
}
static int config_validate(const struct config *c){
    if(c->setpoint<40 || c->setpoint>65 || c->off_hyst<0.5 || c->off_hyst>10 ||
       c->off_confirm<1 || c->off_confirm>180 || c->sample_s<0.5 || c->sample_s>5 ||
       c->log_period<1 || c->log_period>3600 ||
       c->envelope_s<1 || c->envelope_s>60 || c->filter_tau<0 || c->filter_tau>180 ||
       c->kp<0 || c->kp>300 || c->ki<0 || c->ki>30 ||
       c->pwm_up_rate<1 || c->pwm_down_rate<1 ||
       c->pwm_up_rate>255 || c->pwm_down_rate>255 ||
       c->kick_s<0.2 || c->kick_s>5 ||
       c->pwm_start<1 || c->pwm_start>255 || c->pwm_hold<1 || c->pwm_hold>255 ||
       c->pwm_max<1 || c->pwm_max>255 || c->pwm_hold>c->pwm_max ||
       c->pwm_start>c->pwm_max || c->pwm_start<c->pwm_hold ||
       c->protection_hyst<1 || c->protection_hyst>15 ||
       c->protection_recover_s<5 || c->protection_recover_s>600){
        fprintf(stderr,"Configuration: invalid control parameter\n");return -1;
    }
    /* Software guard ceilings, not a claim of manufacturer throttle temperatures. */
    static const double ceilings[GROUPS]={0,95,82,95,88,92,68};
    for(int g=CPU;g<GROUPS;g++){
        if(c->limits[g]<45 || c->limits[g]>ceilings[g]){
            fprintf(stderr,"Configuration: %s protection limit must be 45..%.0f C\n",group_names[g],ceilings[g]);return -1;
        }
    }
    if(c->setpoint>=c->limits[CPU]-5 || c->setpoint>=c->limits[GPU]-5 || c->setpoint>=c->limits[NPU]-5){
        fprintf(stderr,"Configuration: setpoint must remain >=5 C below CPU/GPU/NPU protection limits\n");return -1;
    }
    return 0;
}
static int config_load(const char *path,struct config *out){
    struct config x=defaults;
    FILE *f=fopen(path,"r");if(!f){perror(path);return -1;}
    char line[512];int lineno=0,rc=-1;
    while(fgets(line,sizeof(line),f)){
        lineno++;
        if(!strchr(line,'\n') && !feof(f)){fprintf(stderr,"Long config line %d\n",lineno);goto done;}
        char *comment=strpbrk(line,"#;");if(comment)*comment=0;
        char *p=trim(line);if(!*p)continue;
        char *eq=strchr(p,'=');if(!eq){fprintf(stderr,"Invalid config line %d\n",lineno);goto done;}
        *eq=0;char *key=trim(p),*raw=trim(eq+1),*end;
        errno=0;double val=strtod(raw,&end);
        if(errno || end==raw || *trim(end) || !isfinite(val)){
            fprintf(stderr,"Invalid number on line %d\n",lineno);goto done;
        }
#define SETVAL(name,field) if(!strcmp(key,name))x.field=val
#define SETINT(name,field) if(!strcmp(key,name)){if(val!=trunc(val)||val<INT_MIN||val>INT_MAX){fprintf(stderr,"Noninteger or out-of-range %s\n",key);goto done;} x.field=(int)val;}
        SETVAL("setpoint_c",setpoint);
        else SETVAL("off_hysteresis_c",off_hyst);
        else SETVAL("off_confirm_s",off_confirm);
        else SETVAL("sample_s",sample_s);
        else SETVAL("log_period_s",log_period);
        else SETVAL("envelope_s",envelope_s);
        else SETVAL("filter_tau_s",filter_tau);
        else SETVAL("kp",kp);
        else SETVAL("ki",ki);
        else SETVAL("pwm_up_per_s",pwm_up_rate);
        else SETVAL("pwm_down_per_s",pwm_down_rate);
        else SETVAL("kick_s",kick_s);
        else SETINT("pwm_start",pwm_start)
        else SETINT("pwm_hold",pwm_hold)
        else SETINT("pwm_max",pwm_max)
        else SETVAL("protection_hysteresis_c",protection_hyst);
        else SETVAL("protection_recovery_s",protection_recover_s);
        else SETVAL("cpu_limit_c",limits[CPU]);
        else SETVAL("gpu_limit_c",limits[GPU]);
        else SETVAL("npu_limit_c",limits[NPU]);
        else SETVAL("memory_limit_c",limits[MEM]);
        else SETVAL("pmic_limit_c",limits[PMIC]);
        else SETVAL("vbat_limit_c",limits[VBAT]);
        else {fprintf(stderr,"Unknown config key '%s' at line %d\n",key,lineno);goto done;}
#undef SETVAL
#undef SETINT
    }
    if(ferror(f)){perror("read config");goto done;}
    if(config_validate(&x))goto done;
    *out=x;rc=0;
done:
    fclose(f);return rc;
}

struct sample {double temp[MAX_ZONES], max[GROUPS], control_raw;const char *control_name;int fault;const char *fault_name;double fault_temp,fault_limit;};
static int sample_sensors(struct sample *s){
    for(int g=0;g<GROUPS;g++)s->max[g]=-INFINITY;
    s->fault=0;s->fault_name=NULL;s->fault_temp=NAN;s->fault_limit=NAN;s->control_name="-";s->control_raw=-INFINITY;
    for(int i=0;i<nzones;i++){
        long millic;
        if(read_long(zones[i].path,&millic) || millic<-40000 || millic>150000){
            s->temp[i]=NAN;
            if(zones[i].group!=OTHER){s->fault=1;s->fault_name=zones[i].name;}
            continue;
        }
        double t=millic/1000.;s->temp[i]=t;
        int g=zones[i].group;
        if(t>s->max[g])s->max[g]=t;
        if((g==CPU || g==GPU || g==NPU) && t>s->control_raw){s->control_raw=t;s->control_name=zones[i].name;}
        if(g!=OTHER && (t<=0 || t>=cfg.limits[g])){s->fault=2;s->fault_name=zones[i].name;s->fault_temp=t;s->fault_limit=cfg.limits[g];}
    }
    if(!isfinite(s->max[CPU]) || !isfinite(s->max[GPU]) || !isfinite(s->control_raw))s->fault=1;
    return s->fault;
}

static int set_fan_pwm(int pwm){
    if(pwm<0 || pwm>255)return -1;
    if(!mode_manual){if(write_long(mode_path,1))return -1;mode_manual=1;}
    if(commanded_pwm!=pwm){if(write_long(pwm_path,pwm))return -1;commanded_pwm=pwm;}
    long read_pwm,read_mode;
    if(read_long(mode_path,&read_mode) || read_mode!=1 || read_long(pwm_path,&read_pwm) || read_pwm!=pwm)return -1;
    return 0;
}
static int restore_auto(void){
    if(write_long(mode_path,2))return -1;
    long mode;if(read_long(mode_path,&mode) || mode!=2)return -1;
    mode_manual=0;commanded_pwm=-1;return 0;
}
static int acquire_lock(void){
    lock_fd=open(LOCK_PATH,O_CREAT|O_RDWR|O_CLOEXEC,0600);
    if(lock_fd<0 || flock(lock_fd,LOCK_EX|LOCK_NB)<0){
        fprintf(stderr,"Cannot acquire %s: another fan controller or calibration may be running\n",LOCK_PATH);return -1;
    }
    return 0;
}
static void guardian_main(int rfd,pid_t parent){
    /* Main writes one heartbeat each sampling cycle. EOF/timeout = restore ADSP. */
    struct pollfd p={.fd=rfd,.events=POLLIN|POLLHUP};char buf[64];
    for(;;){
        int n=poll(&p,1,GUARD_TIMEOUT_MS);
        if(n>0){ssize_t k=read(rfd,buf,sizeof(buf));if(k>0)continue;break;}
        if(n==0){kill(parent,SIGKILL);break;}
        if(errno==EINTR)continue;
        break;
    }
    (void)write_long(mode_path,2);
    close(rfd);
    _exit(0);
}
static int guardian_start(void){
    int fds[2];if(pipe2(fds,O_CLOEXEC)<0)return -1;
    pid_t parent=getpid(),p=fork();
    if(p<0){close(fds[0]);close(fds[1]);return -1;}
    if(p==0){close(fds[1]);if(lock_fd>=0)close(lock_fd);if(events)fclose(events);guardian_main(fds[0],parent);}
    close(fds[0]);guardian_pid=p;guardian_fd=fds[1];fan_owned=1;
    return 0;
}
static int heartbeat(void){
    char ping='.';
    if(guardian_fd<0)return -1;
    ssize_t r=write(guardian_fd,&ping,1);
    return r==1?0:-1;
}
static void guardian_stop(void){
    if(guardian_fd>=0){close(guardian_fd);guardian_fd=-1;}
    if(guardian_pid>0){int status;while(waitpid(guardian_pid,&status,0)<0 && errno==EINTR){}guardian_pid=-1;}
}
static int wait_s(double delay){
    double until=now_mono()+delay;
    while(!terminate_requested){
        double rem=until-now_mono();if(rem<=0)break;
        double x=fmin(rem,1.0);
        struct timespec ts={(time_t)x,(long)((x-floor(x))*1.e9)};
        if(nanosleep(&ts,NULL)<0 && errno!=EINTR)return -1;
        if(fan_owned && heartbeat())return -1;
    }
    return terminate_requested?-1:0;
}
static int log_dir_prepare(void){
    if(mkdir(LOG_DIR,0750)<0 && errno!=EEXIST){perror(LOG_DIR);return -1;}
    struct stat st;if(stat(LOG_DIR,&st)<0 || !S_ISDIR(st.st_mode)){fprintf(stderr,"Log directory invalid\n");return -1;}
    return 0;
}
/* Normal operation: only sparse events/errors, never per-sample CSV.
 * journalctl receives stderr when running under systemd; events.log is for
 * manual runs. Both have exactly the same content.
 */
static void log_event(const char *fmt, ...){
    char msg[640];
    va_list ap;va_start(ap,fmt);vsnprintf(msg,sizeof(msg),fmt,ap);va_end(ap);
    time_t wall=time(NULL);struct tm tmv;localtime_r(&wall,&tmv);
    char ts[32];strftime(ts,sizeof(ts),"%Y-%m-%d %H:%M:%S",&tmv);
    fprintf(stderr,"%s %s\n",ts,msg);
    if(events){
        fprintf(events,"%s %s\n",ts,msg);
        if(fflush(events))fprintf(stderr,"Event log write failed: %s\n",strerror(errno));
    }
}
static void open_event_log(void){
    char path[PATH_MAX];
    if(join(path,sizeof(path),LOG_DIR,"events.log"))return;
    events=fopen(path,"a");
    if(!events)fprintf(stderr,"Cannot open %s: %s (continuing via stderr)\n",path,strerror(errno));
}
static void close_logs(void){
    if(csv){fclose(csv);csv=NULL;}
    if(events){fclose(events);events=NULL;}
}
static int open_csv_if_needed(void){
    time_t wall=time(NULL);struct tm tmv;localtime_r(&wall,&tmv);
    char day[16];strftime(day,sizeof(day),"%Y%m%d",&tmv);
    if(csv && !strcmp(current_day,day))return 0;
    if(csv){fclose(csv);csv=NULL;}
    snprintf(current_day,sizeof(current_day),"%s",day);
    char filename[80];snprintf(filename,sizeof(filename),"q8b-fan-%s-%ld.csv",day,(long)getpid());
    if(join(csv_path,sizeof(csv_path),LOG_DIR,filename))return -1;
    csv=fopen(csv_path,"a");if(!csv){perror(csv_path);return -1;}
    if(ftell(csv)==0){
        fprintf(csv,"epoch_ms,elapsed_s,state,phase,control_sensor,control_raw_c,envelope_c,feedback_c,error_c,integrator,pwm_command,pwm_read,pwm_mode");
        for(int g=CPU;g<GROUPS;g++)fprintf(csv,",%s_max_c",group_names[g]);
        for(int i=0;i<nzones;i++)fprintf(csv,",temp_%s_c",zones[i].csv_name);
        for(int i=0;i<8;i++)fprintf(csv,",cpu%d_freq_khz",i);
        fputc('\n',csv);if(fflush(csv))return -1;
    }
    fprintf(stderr,"CSV: %s\n",csv_path);
    return 0;
}
static int write_csv(const struct sample *s,enum state state,const char *phase,double envelope,double feedback,double error,double integral){
    if(open_csv_if_needed())return -1;
    struct timespec t;clock_gettime(CLOCK_REALTIME,&t);
    long long epoch=(long long)t.tv_sec*1000+t.tv_nsec/1000000;
    long pwm=-1,mode=-1;(void)read_long(pwm_path,&pwm);(void)read_long(mode_path,&mode);
    fprintf(csv,"%lld,%.3f,%s,%s,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%ld,%ld",
            epoch,now_mono()-start_at,state_names[state],phase,s->control_name,s->control_raw,envelope,feedback,error,integral,commanded_pwm,pwm,mode);
    for(int g=CPU;g<GROUPS;g++){
        if(isfinite(s->max[g]))fprintf(csv,",%.3f",s->max[g]);else fputc(',',csv);
    }
    for(int i=0;i<nzones;i++){
        if(isfinite(s->temp[i]))fprintf(csv,",%.3f",s->temp[i]);else fputc(',',csv);
    }
    for(int i=0;i<8;i++){
        char path[PATH_MAX];long frequency;
        snprintf(path,sizeof(path),"/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq",i);
        if(read_long(path,&frequency))fputc(',',csv);
        else fprintf(csv,",%ld",frequency);
    }
    fputc('\n',csv);
    return fflush(csv)==0 && !ferror(csv)?0:-1;
}
static int all_below_recovery(const struct sample *s){
    for(int g=CPU;g<GROUPS;g++){
        if(isfinite(s->max[g]) && s->max[g]>cfg.limits[g]-cfg.protection_hyst)return 0;
    }
    return 1;
}
static int apply_reloaded_config(double *integral, double feedback, enum state state){
    struct config updated;
    if(config_load(config_path,&updated)){log_event("CONFIG rejected: invalid config; old settings retained");return -1;}
    /* Bumpless PI reload: preserve current output by adjusting integral state. */
    if(state==ACTIVE && commanded_pwm>=0)
        *integral=fmax(-updated.pwm_max,fmin(updated.pwm_max,
                      commanded_pwm-updated.kp*(feedback-updated.setpoint)));
    else if(state==OFF)
        *integral=updated.pwm_hold;
    cfg=updated;
    log_event("CONFIG applied: setpoint=%.1fC kp=%.3f ki=%.4f",cfg.setpoint,cfg.kp,cfg.ki);
    return 0;
}
static int run_controller(void){
    if(log_dir_prepare() || acquire_lock() || guardian_start())return 1;
    open_event_log();
    start_at=now_mono();last_tick=start_at;last_log=start_at-cfg.log_period;
    log_event("START setpoint=%.1fC kp=%.3f ki=%.4f record=%s",cfg.setpoint,cfg.kp,cfg.ki,record_enabled?"on":"off");
    struct sample s;
    if(sample_sensors(&s)){
        log_event("ERROR preflight thermal measurement: %s",s.fault_name?s.fault_name:"unknown");goto error;
    }
    if(set_fan_pwm(0)){log_event("ERROR cannot claim manual PWM control");goto error;}
    enum state st=OFF,last_state=OFF;
    double history_t[MAX_HISTORY]={0},history_v[MAX_HISTORY]={0};int n_hist=0,head=0;
    double feedback=s.control_raw,integral=cfg.pwm_hold;
    double below_since=-1,recovery_since=-1,kick_until=-1;
    while(!terminate_requested){
        double t=now_mono(),dt=t-last_tick;last_tick=t;
        if(dt<0)dt=cfg.sample_s;
        if(heartbeat()){log_event("ERROR guardian heartbeat failed");goto error;}
        int fault=sample_sensors(&s);
        if(fault==1){log_event("ERROR sensor failure: %s; returning to ADSP",s.fault_name?s.fault_name:"unknown");goto error;}
        if(fault==2 && st!=PROTECT){
            log_event("PROTECT triggered: %s %.1fC (limit %.1fC); CPU=%.1fC GPU=%.1fC",s.fault_name?s.fault_name:"unknown",s.fault_temp,s.fault_limit,s.max[CPU],s.max[GPU]);
            st=PROTECT;recovery_since=-1;
        }
        history_t[head]=t;history_v[head]=s.control_raw;
        head=(head+1)%MAX_HISTORY;if(n_hist<MAX_HISTORY)n_hist++;
        double envelope=s.control_raw;
        for(int i=0;i<n_hist;i++)if(t-history_t[i]<=cfg.envelope_s && history_v[i]>envelope)envelope=history_v[i];
        double alpha=cfg.filter_tau==0?1.:1.-exp(-fmin(dt,60.)/cfg.filter_tau);
        feedback+=alpha*(envelope-feedback);
        if(reload_requested){reload_requested=0;apply_reloaded_config(&integral,feedback,st);}
        double err=feedback-cfg.setpoint;
        if(st==OFF && feedback>=cfg.setpoint){
            st=ACTIVE;integral=fmax((double)cfg.pwm_hold,integral);
            kick_until=t+cfg.kick_s;below_since=-1;
        }
        if(st==ACTIVE){
            if(feedback<=cfg.setpoint-cfg.off_hyst){
                if(below_since<0)below_since=t;
                if(t-below_since>=cfg.off_confirm){st=OFF;below_since=-1;integral=cfg.pwm_hold;}
            }else below_since=-1;
        }
        if(st==PROTECT){
            if(fault==0 && all_below_recovery(&s)){
                if(recovery_since<0)recovery_since=t;
                if(t-recovery_since>=cfg.protection_recover_s){
                    st=ACTIVE;recovery_since=-1;integral=cfg.pwm_max;
                    kick_until=-1;below_since=-1;
                    log_event("PROTECT cleared: returning to PI");
                }
            }else recovery_since=-1;
        }
        int target=0;
        if(st==ACTIVE){
            double unsat=cfg.kp*err+integral;
            if(!((unsat>=cfg.pwm_max && err>0) || (unsat<=cfg.pwm_hold && err<0))){
                integral+=cfg.ki*err*fmin(dt,10.);
                integral=fmax(-cfg.pwm_max,fmin(cfg.pwm_max,integral));
            }
            double bounded=fmax((double)cfg.pwm_hold,fmin((double)cfg.pwm_max,cfg.kp*err+integral));
            target=(int)lround(bounded);
            if(t<kick_until)target=cfg.pwm_start;
            else if(commanded_pwm>=0){
                /* A hard safety override bypasses both rate limiters. */
                double upper=commanded_pwm+cfg.pwm_up_rate*fmin(dt,10.);
                double lower=commanded_pwm-cfg.pwm_down_rate*fmin(dt,10.);
                if(target>upper)target=(int)ceil(upper);
                if(target<lower)target=(int)floor(lower);
                if(target<cfg.pwm_hold)target=cfg.pwm_hold;
            }
        }else if(st==PROTECT){target=cfg.pwm_max;}
        if(set_fan_pwm(target)){
            log_event("ERROR PWM write/readback failed; returning to ADSP");goto error;
        }
        if(st!=last_state){
            log_event("STATE %s -> %s Traw=%.1fC feedback=%.1fC PWM=%d",
                state_names[last_state],state_names[st],s.control_raw,feedback,target);
            last_state=st;
        }
        if(record_enabled && (t-last_log>=cfg.log_period || fault)){
            if(write_csv(&s,st,"control",envelope,feedback,err,integral)){
                log_event("ERROR CSV write failed: returning to ADSP");goto error;
            }
            last_log=t;
        }
        if(wait_s(cfg.sample_s))break;
    }
    log_event("STOP requested; restoring ADSP automatic fan control");
    if(restore_auto()){log_event("ERROR cannot restore ADSP automatic mode");goto error;}
    log_event("STOP ADSP auto restored");
    close_logs();guardian_stop();return 0;
error:
    if(fan_owned){
        /* Even if max-PWM write fails, firmware automatic control is attempted. */
        (void)set_fan_pwm(255);
        if(restore_auto())log_event("CRITICAL failed to restore ADSP automatic fan control!");
        else log_event("FAILSAFE ADSP auto restored");
    }
    close_logs();guardian_stop();return 1;
}
static int calibration_stage(const char *name,int pwm,double seconds){
    log_event("CAL %-20s PWM=%3d duration=%.0fs",name,pwm,seconds);
    if(set_fan_pwm(pwm))return -1;
    double begin=now_mono();
    while(!terminate_requested && now_mono()-begin<seconds){
        struct sample s;int fault=sample_sensors(&s);
        if(fault){log_event("CAL error thermal/sensor guard: %s",s.fault_name?s.fault_name:"unknown");return -1;}
        if(record_enabled && write_csv(&s,CAL,name,s.control_raw,s.control_raw,0,0))return -1;
        if(wait_s(1.))return -1;
    }
    return terminate_requested?-1:0;
}
static int run_calibration(void){
    if(log_dir_prepare() || acquire_lock() || guardian_start())return 1;
    open_event_log();
    start_at=now_mono();
    log_event("CAL START record=%s; observe fan visually (no RPM sensor)",record_enabled?"on":"off");
    static const int hold_steps[]={160,140,120,100,80,60,40};
    static const int start_steps[]={60,80,100,120,140,160,180};
    int err=0;
    if(calibration_stage("zero baseline",0,12))err=1;
    if(!err && calibration_stage("kick",180,2))err=1;
    for(size_t i=0;!err && i<sizeof(hold_steps)/sizeof(hold_steps[0]);i++){
        char label[48];snprintf(label,sizeof(label),"hold descending %d",hold_steps[i]);
        if(calibration_stage(label,hold_steps[i],12))err=1;
    }
    for(size_t i=0;!err && i<sizeof(start_steps)/sizeof(start_steps[0]);i++){
        char label[48];snprintf(label,sizeof(label),"direct start %d",start_steps[i]);
        if(calibration_stage("stop between starts",0,12))err=1;
        if(!err && calibration_stage(label,start_steps[i],12))err=1;
    }
    if(restore_auto())log_event("CAL ERROR ADSP restore failed");
    log_event("CAL %s; note lowest continuous HOLD and lowest reliable START PWM",err?"STOPPED":"DONE");
    close_logs();guardian_stop();
    return err?1:0;
}
static int probe(void){
    if(discover())return 1;
    for(int i=0;i<nzones;i++){
        if(zones[i].group==OTHER)continue;
        long t;int ok=read_long(zones[i].path,&t);
        printf("%-28s %-6s %6.1f C%s\n",zones[i].name,group_names[zones[i].group],ok?0:t/1000.,ok?" [READ FAIL]":"");
    }
    return 0;
}
static int restore_command(void){
    if(discover())return 1;
    /* Cannot override an active controller: acquire lock first. */
    if(acquire_lock())return 1;
    return restore_auto()?1:0;
}
static void help(void){
    puts("Usage: q8b-fan --probe | --run [CONFIG] [--record] | --calibrate [CONFIG] [--record] | --restore\n"
         "Default: only events/errors in " LOG_DIR "/events.log and stderr.\n"
         "--record: enable detailed CSV at log_period_s interval.\n"
         "Reload settings using SIGHUP; Ctrl+C restores ADSP automatic mode.");
}
int main(int argc,char **argv){
    signal(SIGPIPE,SIG_IGN);
    if(argc==2 && !strcmp(argv[1],"--help")){help();return 0;}
    if(argc==2 && !strcmp(argv[1],"--probe"))return probe();
    if(geteuid()!=0){fprintf(stderr,"Run as root\n");return 2;}
    if(argc==2 && !strcmp(argv[1],"--restore"))return restore_command();
    if(argc<2 || (strcmp(argv[1],"--run") && strcmp(argv[1],"--calibrate"))){help();return 2;}
    const char *selected_config=CONFIG_DEFAULT;int config_seen=0;
    for(int i=2;i<argc;i++){
        if(!strcmp(argv[i],"--record")){
            if(record_enabled){fprintf(stderr,"Duplicate --record\n");return 2;}
            record_enabled=1;
        }else if(argv[i][0]!='-' && !config_seen){
            selected_config=argv[i];config_seen=1;
        }else{fprintf(stderr,"Unexpected argument: %s\n",argv[i]);help();return 2;}
    }
    if(snprintf(config_path,sizeof(config_path),"%s",selected_config)>=(int)sizeof(config_path)){
        fprintf(stderr,"Config path is too long\n");return 2;
    }
    if(config_load(config_path,&cfg))return 2;
    if(discover())return 2;
    struct sigaction sa={0};sa.sa_handler=handle_signal;sigemptyset(&sa.sa_mask);
    sigaction(SIGINT,&sa,NULL);sigaction(SIGTERM,&sa,NULL);sigaction(SIGQUIT,&sa,NULL);
    sigaction(SIGHUP,&sa,NULL);
    if(!strcmp(argv[1],"--calibrate"))return run_calibration();
    return run_controller();
}
