/**
 * @file
 * Takes a log file on stdin, spews it in real-time to a UDP port.
 *
 * Usage: logspew hostname port acceleration
 * e.g. logspew localhost 12312 2 <capture.log
 * for 2* acceleration.
 * 
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include "list_standalone.h"
#include "udpclient.h"


unsigned long curtime=0;
double accel=1;
int port;
const char *host;

char fieldNames[256][256];
int numFields=0;

int findOrCreateField(const char *name){
    for(int i=0;i<numFields;i++){
        if(!strcmp(fieldNames[i],name))
            return i;
    }
    if(numFields==256){
        printf("Too many fields.\n");
        exit(1);
    }
//    printf("Adding field %s\n",name);
    strcpy(fieldNames[numFields],name);
    return numFields++;
}


struct Line {
    unsigned long time;
    int keys[256];
    double vals[256];
    int num;
    ListNode<Line> listnodes[1];
    
    Line(){
//        printf("Creating new line\n");
        num=0;
    }
    
    const char *format(){
        static char buf[1024];
        buf[0]=0;
        int t = findOrCreateField("time");
        for(int i=0;i<num;i++){
            sprintf(buf+strlen(buf),"%s%s=%f",i?" ":"",fieldNames[keys[i]],vals[i]);
        }
        return buf;
    }
    
    bool hasField(int f){
        for(int i=0;i<num;i++)
            if(keys[i]==f)return true;
        return false;
    }
    
    bool hasTime(){
        int t = findOrCreateField("time");
        return hasField(t);
    }
    
    double get(int f){
        for(int i=0;i<num;i++)
            if(keys[i]==f)return vals[i];
        return -1;
    }
    
    void add(int f,double v){
        for(int i=0;i<num;i++)
            if(keys[i]==f){
                printf("two values for field %s in a line\n",fieldNames[f]);
                exit(1);
            }
//        printf("Setting field %d (%s) to %f\n",f,fieldNames[f],v);
        keys[num]=f;
        vals[num++]=v;
    }
        
};

LinkedList<Line,0> list;

void processLine(char *s){
    // tokenise and process
    
    Line *line = new Line();
    
    for(;;){
        char *t;
        if(!(t = strtok(s," ")))
            break;
        s=NULL;
        
        char *q = strchr(t,'=');
        
        if(q){
            *q++ = 0;
            
            int f = findOrCreateField(t);
            line->add(f,atof(q));
        }
    }
    if(line->hasTime())
        list.addToTail(line);
    else
        delete line;
}


double gettime(){
    timespec ts;
    clock_gettime(CLOCK_REALTIME,&ts);
    
    double t = ts.tv_nsec;
    t *= 1e-9;
    t += ts.tv_sec;
    return t;
}

void outputList(){
    int fieldTime = findOrCreateField("time");
    double startTime = gettime();
    double firstTime = list.head()->get(fieldTime);
    
    for(Line *p=list.head();p;p=list.next(p)){
        double nextItemTime = p->get(fieldTime) - firstTime;
        nextItemTime *= accel;
        nextItemTime += 2;
        
        for(;;){
            usleep(10000);
            double t = gettime()-startTime;
            if(t>nextItemTime)break;
        }
        
        udpSend(host,port,p->format());
    }
    
}



int main(int argc,char *argv[]){
    
    char buf[1024];
    
    if(argc<4){
        printf("Usage: logspew PORT\n");
        exit(1);
    }
    
    host = argv[1];
    port = atoi(argv[2]);
    accel = 1.0/atof(argv[3]);
    
    udpInit();
    
    for(int i=0;i<256;i++)
        fieldNames[i][0]=0;
    
    for(;;){
        if(feof(stdin))
            break;
        fgets(buf,1024,stdin);
        processLine(buf);
    }
    
    outputList();
    return 0;
}
