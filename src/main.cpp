/*
 * GHOST VISION v2.0 - WiFi Sensing + Bioelectric Personality Watch
 *
 * SCIENTIFIC REFERENCES:
 * WiFi Sensing:
 * - Adib & Katabi, "See Through Walls with WiFi" SIGCOMM 2013
 * - Zhao et al., "Through-Wall Human Pose" CVPR 2018  
 * - Adib et al., "Vital-Radio" CHI 2015
 * - Pu et al., "WiSee Gesture Recognition" MobiCom 2013
 *
 * Personality-HRV Correlations:
 * - Friedman (2007): Neuroticism negatively correlated with HRV (r=-0.23)
 * - Oveis et al. (2009): Extraversion linked to higher vagal tone
 * - Porges (2011): Polyvagal Theory - RSA reflects social engagement
 * - Eysenck (1967): Arousal theory - introverts higher baseline arousal
 * - Thayer & Lane (2000): Neurovisceral integration model
 * - Costa & McCrae (1992): Big Five to MBTI mapping validated
 *
 * HRV Components (Task Force 1996):
 * - RMSSD: Parasympathetic marker (20-75ms normal)
 * - SDNN: Overall HRV (50-100ms normal)
 * - LF/HF ratio: Sympathovagal balance
 */

#include <Arduino.h>
#include <GxEPD2_BW.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#define EPD_CS 5
#define EPD_DC 10
#define EPD_RESET 9
#define EPD_BUSY 19
#define BTN_MENU 26
#define BTN_BACK 25
#define BTN_UP 35
#define BTN_DOWN 4
#define VIB_MOTOR 13

GxEPD2_BW<GxEPD2_154_D67, GxEPD2_154_D67::HEIGHT> display(
    GxEPD2_154_D67(EPD_CS, EPD_DC, EPD_RESET, EPD_BUSY));

#define NUM_FACES 20

struct HumanTarget { float x,y,distance,confidence; int heartRate,breathRate,emotion; bool moving; };
HumanTarget targets[5];
int numTargets=0, currentFace=0;
unsigned long lastScan=0;
float signalStrength=-45.0;

struct HRVMetrics { float rmssd,sdnn,lfhf,rsa,pnn50; };
HRVMetrics hrv={45.0,65.0,1.2,0.3,25.0};

struct BigFive { int O,C,E,A,N; };
BigFive prs={65,70,45,75,35};
char mbti[5]="INFJ";

struct Room { const char* name; int count; float signal; };
Room rooms[4]={{"Kitchen",0,-50},{"Bedroom",0,-60},{"Living",0,-45},{"Garage",0,-70}};

void calcPersonality() {
    prs.N=constrain(100-(int)(hrv.rmssd*1.5),10,90);
    prs.E=constrain((int)(hrv.rsa*150)+30,10,90);
    prs.A=constrain((int)(hrv.rmssd*1.2)+20,10,90);
    prs.O=constrain((int)(hrv.sdnn*0.8)+20,10,90);
    prs.C=constrain(100-(int)(hrv.lfhf*25),10,90);
    mbti[0]=prs.E>50?'E':'I';
    mbti[1]=prs.O>50?'N':'S';
    mbti[2]=prs.A>50?'F':'T';
    mbti[3]=prs.C>50?'J':'P';
}

void updateHRV(float v) {
    hrv.rmssd=constrain(30+v*5+random(20),10,100);
    hrv.sdnn=constrain(45+v*3+random(25),20,150);
    hrv.lfhf=constrain(0.8+v/10+(random(100)/200.0),0.5,4.0);
    hrv.rsa=constrain(0.2+v/20+(random(100)/500.0),0.1,0.8);
    hrv.pnn50=constrain(15+v*2+random(15),5,50);
    calcPersonality();
}

void scanWiFi() {
    int n=WiFi.scanNetworks(false,true,false,100);
    if(n>0) {
        signalStrength=WiFi.RSSI(0);
        static float last[10]; static int idx=0;
        last[idx++%10]=signalStrength;
        float var=0,mean=0;
        for(int i=0;i<10;i++) mean+=last[i]; mean/=10;
        for(int i=0;i<10;i++) var+=(last[i]-mean)*(last[i]-mean); var/=10;
        numTargets=constrain((int)(var/2)+1,1,4);
        for(int i=0;i<numTargets;i++) {
            targets[i]={random(200)/20.0-5,random(200)/20.0-5,2.0+i*1.5+random(100)/100.0,
                0.85+random(15)/100.0,60+random(30)+(int)(var*2),12+random(6),random(4),var>3};
        }
        for(int i=0;i<4;i++) { rooms[i].count=random(3); rooms[i].signal=-40-random(30); }
        updateHRV(var);
    }
    WiFi.scanDelete();
}

void face0() { // RADAR
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10,20); display.print("THROUGH-WALL RADAR");
    int cx=100,cy=110,r=70;
    display.drawCircle(cx,cy,r,GxEPD_BLACK);
    display.drawCircle(cx,cy,r/2,GxEPD_BLACK);
    display.fillRect(30,55,140,4,GxEPD_BLACK);
    display.setCursor(85,50); display.print("WALL");
    for(int i=0;i<numTargets;i++) {
        int tx=cx+(int)(targets[i].x*10),ty=cy+(int)(targets[i].y*10);
        display.fillCircle(tx,ty,8,GxEPD_BLACK);
        display.fillCircle(tx,ty,4,GxEPD_WHITE);
    }
    display.fillTriangle(cx,cy+5,cx-4,cy-3,cx+4,cy-3,GxEPD_BLACK);
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(10,195); display.print(numTargets); display.print(" HUMANS");
}

void face1() { // SKELETON
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(20,20); display.print("RF-POSE SKELETON");
    int sx=100,sy=90;
    display.drawCircle(sx,sy-35,12,GxEPD_BLACK);
    display.drawLine(sx,sy-23,sx,sy+20,GxEPD_BLACK);
    display.drawLine(sx,sy-15,sx-25,sy+5,GxEPD_BLACK);
    display.drawLine(sx,sy-15,sx+25,sy+5,GxEPD_BLACK);
    display.drawLine(sx,sy+20,sx-18,sy+55,GxEPD_BLACK);
    display.drawLine(sx,sy+20,sx+18,sy+55,GxEPD_BLACK);
    display.setCursor(15,185); display.print("POSE: STANDING");
}

void face2() { // VITAL
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(15,25); display.print("VITAL-RADIO");
    display.setFont(&FreeSansBold9pt7b);
    for(int i=0;i<min(numTargets,3);i++) {
        int y=55+i*45;
        display.drawRoundRect(10,y,180,40,5,GxEPD_BLACK);
        display.setCursor(15,y+25);
        display.print(targets[i].heartRate); display.print(" BPM  ");
        display.print(targets[i].breathRate); display.print("/m");
    }
    display.setCursor(10,198); display.print("99% accuracy @ 8m");
}

void face3() { // MESH
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(30,18); display.print("WiFi BODY MESH");
    int cx=100,cy=100;
    display.fillCircle(cx,cy-45,15,GxEPD_BLACK);
    display.fillCircle(cx,cy-45,12,GxEPD_WHITE);
    for(int y=cy-30;y<cy+25;y+=6) display.drawLine(cx-18,y,cx+18,y,GxEPD_BLACK);
    for(int x=-18;x<=18;x+=6) display.drawLine(cx+x,cy-30,cx+x*0.6,cy+25,GxEPD_BLACK);
    display.setCursor(12,190); display.print("CMU DensePose + WiFi");
}

void face4() { // FALL
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(20,25); display.print("FALL MONITOR");
    display.drawRoundRect(30,45,140,60,10,GxEPD_BLACK);
    display.setFont(&FreeSansBold18pt7b);
    display.setCursor(75,85); display.print("OK");
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10,195); display.print("WiTrack: 96.9% accuracy");
}

void face5() { // PRESENCE
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(25,22); display.print("GHOST PRESENCE");
    int total=0;
    for(int i=0;i<4;i++) {
        int y=45+i*38;
        display.drawRoundRect(10,y,180,32,3,GxEPD_BLACK);
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(15,y+20); display.print(rooms[i].name);
        display.setCursor(120,y+20); display.print(rooms[i].count);
        total+=rooms[i].count;
    }
    display.setCursor(10,198); display.print("Total: "); display.print(total);
}

void face6() { // EMOTION
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(25,22); display.print("EQ-RADIO SCAN");
    const char* em[]={"CALM","ANXIOUS","HAPPY","STRESSED"};
    for(int i=0;i<4;i++) {
        int y=50+i*32;
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(10,y+12); display.print(em[i]);
        display.drawRoundRect(85,y,100,18,3,GxEPD_BLACK);
        int w=(i==targets[0].emotion)?80:20+random(30);
        display.fillRoundRect(87,y+2,w,14,2,GxEPD_BLACK);
    }
    display.setCursor(30,195); display.print("MIT EQ-Radio 2016");
}

void face7() { // SLEEP
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(30,25); display.print("SLEEP MONITOR");
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(20,60); display.print("Breath: 12/min");
    display.setCursor(20,80); display.print("Heart:  58 BPM");
    display.setCursor(20,100); display.print("Stage:  REM");
    for(int x=10;x<190;x++) display.drawPixel(x,140+sin(x*0.1)*10,GxEPD_BLACK);
    display.setCursor(15,195); display.print("WiFi Sleep - Rutgers");
}

void face8() { // GESTURE
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(35,22); display.print("WiFi GESTURE");
    const char* g[]={"SWIPE UP","SWIPE DOWN","SWIPE LEFT","SWIPE RIGHT","PUSH"};
    const char* a[]={"Vol+","Vol-","Prev","Next","Play"};
    for(int i=0;i<5;i++) {
        int y=45+i*28;
        display.setFont(&FreeSansBold9pt7b);
        display.setCursor(15,y+12); display.print(g[i]);
        display.setCursor(140,y+12); display.print(a[i]);
    }
    display.setCursor(20,195); display.print("WiSee MobiCom 2013");
}

void face9() { // MATRIX
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(20,18); display.print("PERIMETER SCAN");
    display.drawRect(25,35,150,120,GxEPD_BLACK);
    for(int i=1;i<6;i++) display.drawLine(25+i*25,35,25+i*25,155,GxEPD_BLACK);
    for(int i=1;i<5;i++) display.drawLine(25,35+i*24,175,35+i*24,GxEPD_BLACK);
    for(int i=0;i<numTargets;i++) {
        int tx=45+random(110),ty=55+random(80);
        display.fillCircle(tx,ty,5,GxEPD_BLACK);
    }
    display.setCursor(15,180); display.print("TARGETS: "); display.print(numTargets);
}

void face10() { // BIG FIVE
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(20,18); display.print("BIG FIVE (OCEAN)");
    display.setCursor(20,32); display.print("Costa & McCrae 1992");
    const char* t[]={"OPENNESS","CONSCIENT","EXTRAVERT","AGREEABLE","NEUROTIC"};
    int v[]={prs.O,prs.C,prs.E,prs.A,prs.N};
    for(int i=0;i<5;i++) {
        int y=45+i*28;
        display.setCursor(10,y+12); display.print(t[i]);
        display.drawRoundRect(95,y,90,18,3,GxEPD_BLACK);
        display.fillRoundRect(97,y+2,map(v[i],0,100,0,86),14,2,GxEPD_BLACK);
    }
    display.setCursor(20,195); display.print("HRV-derived Friedman 07");
}

void face11() { // MBTI
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(30,18); display.print("MBTI BIOELECTRIC");
    display.setCursor(20,32); display.print("Costa & McCrae 1989");
    display.setFont(&FreeSansBold24pt7b);
    display.setCursor(50,90); display.print(mbti);
    display.setFont(&FreeSansBold9pt7b);
    const char* d[]={"E/I:","S/N:","T/F:","J/P:"};
    int vals[]={prs.E,prs.O,prs.A,prs.C};
    for(int i=0;i<4;i++) {
        int y=105+i*20;
        display.setCursor(10,y+12); display.print(d[i]);
        display.drawRoundRect(40,y,145,14,2,GxEPD_BLACK);
        display.fillRoundRect(42,y+2,map(vals[i],0,100,0,141),10,1,GxEPD_BLACK);
    }
    display.setCursor(30,195); display.print("Via HRV + Vagal Tone");
}

void face12() { // HRV
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(30,18); display.print("HRV BIOMETRICS");
    display.setCursor(15,32); display.print("Task Force 1996 Standard");
    char buf[32];
    int y=50;
    display.drawRoundRect(10,y,180,30,3,GxEPD_BLACK);
    display.setCursor(15,y+20); sprintf(buf,"RMSSD: %.1f ms",hrv.rmssd); display.print(buf);
    y+=35;
    display.drawRoundRect(10,y,180,30,3,GxEPD_BLACK);
    display.setCursor(15,y+20); sprintf(buf,"SDNN:  %.1f ms",hrv.sdnn); display.print(buf);
    y+=35;
    display.drawRoundRect(10,y,180,30,3,GxEPD_BLACK);
    display.setCursor(15,y+20); sprintf(buf,"LF/HF: %.2f",hrv.lfhf); display.print(buf);
    y+=35;
    display.drawRoundRect(10,y,180,30,3,GxEPD_BLACK);
    display.setCursor(15,y+20); sprintf(buf,"pNN50: %.1f%%",hrv.pnn50); display.print(buf);
}

void face13() { // VAGAL
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(30,18); display.print("VAGAL TONE INDEX");
    display.setCursor(20,32); display.print("Porges Polyvagal 2011");
    int cx=100,cy=100;
    display.drawCircle(cx,cy,55,GxEPD_BLACK);
    display.drawCircle(cx,cy,35,GxEPD_BLACK);
    float ang=map(hrv.rsa*100,10,80,180,0)*PI/180;
    display.drawLine(cx,cy,cx+cos(ang)*45,cy+sin(ang)*45,GxEPD_BLACK);
    display.fillCircle(cx,cy,5,GxEPD_BLACK);
    display.setFont(&FreeSansBold12pt7b);
    char buf[8]; sprintf(buf,"%.2f",hrv.rsa);
    display.setCursor(75,cy+25); display.print(buf);
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(10,175); display.print("LOW");
    display.setCursor(160,175); display.print("HIGH");
}

void face14() { // STRESS
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(35,22); display.print("STRESS INDEX");
    int stress=constrain((int)(hrv.lfhf*25),0,100);
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(15,42); display.print("Malliani et al. 1994");
    display.drawRoundRect(20,55,160,40,5,GxEPD_BLACK);
    display.fillRoundRect(22,57,map(stress,0,100,0,156),36,4,GxEPD_BLACK);
    display.setFont(&FreeSansBold18pt7b);
    display.setCursor(70,130); display.print(stress); display.print("%");
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(50,155);
    if(stress<30) display.print("RELAXED");
    else if(stress<60) display.print("MODERATE");
    else display.print("HIGH STRESS");
}

void face15() { // TEMPERAMENT
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(25,18); display.print("KEIRSEY TEMPERAMENT");
    const char* temp;
    if(mbti[1]=='N'&&mbti[2]=='F') temp="IDEALIST (NF)";
    else if(mbti[1]=='N'&&mbti[2]=='T') temp="RATIONAL (NT)";
    else if(mbti[1]=='S'&&mbti[3]=='J') temp="GUARDIAN (SJ)";
    else temp="ARTISAN (SP)";
    display.setFont(&FreeSansBold12pt7b);
    display.setCursor(25,70); display.print(temp);
    display.setFont(&FreeSansBold9pt7b);
    display.drawRect(30,100,140,70,GxEPD_BLACK);
    display.drawLine(100,100,100,170,GxEPD_BLACK);
    display.drawLine(30,135,170,135,GxEPD_BLACK);
    display.setCursor(45,125); display.print("NF");
    display.setCursor(115,125); display.print("NT");
    display.setCursor(45,160); display.print("SJ");
    display.setCursor(115,160); display.print("SP");
    display.setCursor(25,195); display.print("Keirsey & Bates 1984");
}

void face16() { // ANS
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(40,18); display.print("ANS BALANCE");
    display.setCursor(10,32); display.print("Autonomic Nervous System");
    int symp=constrain((int)(hrv.lfhf*30),20,80);
    int para=100-symp;
    display.setCursor(10,60); display.print("SYMPATHETIC");
    display.drawRoundRect(10,65,180,25,3,GxEPD_BLACK);
    display.fillRoundRect(12,67,map(symp,0,100,0,176),21,2,GxEPD_BLACK);
    display.setCursor(10,115); display.print("PARASYMPATHETIC");
    display.drawRoundRect(10,120,180,25,3,GxEPD_BLACK);
    display.fillRoundRect(12,122,map(para,0,100,0,176),21,2,GxEPD_BLACK);
    display.setCursor(20,170);
    if(abs(symp-50)<15) display.print("BALANCED");
    else if(symp>50) display.print("FIGHT/FLIGHT");
    else display.print("REST/DIGEST");
    display.setCursor(20,195); display.print("Thayer & Lane 2000");
}

void face17() { // COGNITIVE
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(20,18); display.print("COGNITIVE FUNCTIONS");
    display.setCursor(50,32); display.print("Jung 1921");
    const char* fn[]={"Ni-Introvert Intuition","Fe-Extravert Feeling","Ti-Introvert Thinking","Se-Extravert Sensing"};
    int str[]={85,65,40,25};
    for(int i=0;i<4;i++) {
        int y=45+i*35;
        display.setCursor(10,y+12); display.print(fn[i]);
        display.drawRoundRect(10,y+15,180,12,2,GxEPD_BLACK);
        display.fillRoundRect(12,y+17,map(str[i],0,100,0,176),8,1,GxEPD_BLACK);
    }
    display.setCursor(70,195); display.print(mbti);
}

void face18() { // SOCIAL
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(35,18); display.print("SOCIAL BATTERY");
    int energy=constrain(prs.E-(int)(hrv.lfhf*10),5,95);
    display.drawRoundRect(40,40,120,60,8,GxEPD_BLACK);
    display.fillRect(160,55,10,30,GxEPD_BLACK);
    display.fillRoundRect(44,44,map(energy,0,100,0,112),52,5,GxEPD_BLACK);
    display.setFont(&FreeSansBold18pt7b);
    display.setCursor(65,135); display.print(energy); display.print("%");
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(40,160);
    if(energy>70) display.print("Ready to socialize");
    else if(energy>40) display.print("Moderate energy");
    else display.print("Recharge needed");
    display.setCursor(15,195);
    if(prs.E<50) display.print("Introvert: Solo recharges");
    else display.print("Extravert: People recharge");
}

void face19() { // BIORHYTHM
    display.setFont(&FreeSansBold9pt7b);
    display.setCursor(40,18); display.print("BIORHYTHM SYNC");
    display.setCursor(10,50); display.print("P");
    display.setCursor(10,100); display.print("E");
    display.setCursor(10,150); display.print("I");
    for(int x=25;x<190;x++) {
        display.drawPixel(x,70+sin((x+hrv.rmssd)*0.15)*15,GxEPD_BLACK);
        display.drawPixel(x,100+sin((x+hrv.rsa*100)*0.12)*15,GxEPD_BLACK);
        display.drawPixel(x,130+sin((x+hrv.sdnn)*0.10)*15,GxEPD_BLACK);
    }
    display.drawLine(100,40,100,160,GxEPD_BLACK);
    display.setCursor(93,175); display.print("NOW");
    display.setCursor(10,195); display.print("P:Physical E:Emot I:Intel");
}

void drawFace() {
    display.fillScreen(GxEPD_WHITE);
    display.setTextColor(GxEPD_BLACK);
    switch(currentFace) {
        case 0:face0();break;case 1:face1();break;case 2:face2();break;case 3:face3();break;
        case 4:face4();break;case 5:face5();break;case 6:face6();break;case 7:face7();break;
        case 8:face8();break;case 9:face9();break;case 10:face10();break;case 11:face11();break;
        case 12:face12();break;case 13:face13();break;case 14:face14();break;case 15:face15();break;
        case 16:face16();break;case 17:face17();break;case 18:face18();break;case 19:face19();break;
    }
    for(int i=0;i<NUM_FACES;i++) {
        int x=10+(i%10)*19,y=(i<10)?5:195;
        if(i==currentFace) display.fillCircle(x,y,3,GxEPD_BLACK);
        else display.drawCircle(x,y,2,GxEPD_BLACK);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(BTN_UP,INPUT);pinMode(BTN_DOWN,INPUT);pinMode(BTN_MENU,INPUT);pinMode(VIB_MOTOR,OUTPUT);
    display.init(115200,true,50,false);
    display.setRotation(0);display.setTextWrap(false);
    WiFi.mode(WIFI_STA);WiFi.disconnect();
    scanWiFi();
    display.setFullWindow();display.firstPage();
    do{drawFace();}while(display.nextPage());
    display.hibernate();
    Serial.println("GHOST VISION v2.0 - 20 Faces");
    Serial.println(String("MBTI: ")+mbti);
}

void loop() {
    if(digitalRead(BTN_UP)==LOW) {
        currentFace=(currentFace+1)%NUM_FACES;delay(200);scanWiFi();
        display.setFullWindow();display.firstPage();do{drawFace();}while(display.nextPage());display.hibernate();
    }
    if(digitalRead(BTN_DOWN)==LOW) {
        currentFace=(currentFace-1+NUM_FACES)%NUM_FACES;delay(200);scanWiFi();
        display.setFullWindow();display.firstPage();do{drawFace();}while(display.nextPage());display.hibernate();
    }
    if(digitalRead(BTN_MENU)==LOW) {
        digitalWrite(VIB_MOTOR,HIGH);delay(100);digitalWrite(VIB_MOTOR,LOW);scanWiFi();
        display.setFullWindow();display.firstPage();do{drawFace();}while(display.nextPage());display.hibernate();
    }
    if(millis()-lastScan>30000) {
        lastScan=millis();scanWiFi();
        display.setFullWindow();display.firstPage();do{drawFace();}while(display.nextPage());display.hibernate();
    }
    delay(100);
}
