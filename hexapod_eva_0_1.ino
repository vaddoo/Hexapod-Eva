#include <PS2X_lib.h>  //for v1.6
#include <Servo.h> 

//#define NBR_SERVOS 24

Servo coxaServo[6], femurServo[6], tibiaServo[6];

//=================================
//  распиновка
//=================================
const int coxaPin[6] = {
  53, 49, 45, 40, 36, 32};
int femurPin[6] = {
  52, 48, 44, 39, 35, 31};
int tibiaPin[6] = {
  51, 47, 43, 38, 34, 30};

int tonePin = 2; // пин для подключения динамика
int greenLED = 5, yellowLED = 4, redLED = 3; // пины для светодиодов

const int coxaCorr[6] = {  //  калиборовка серв
  -6, -9, 2, 2, 4, -8};
const int femurCorr[6] = {
  -6, -2, 5, 2, -8, -2};
const int tibiaCorr[6] = {
  2, -5, 5, 0, -7, +5};

boolean hexOn = false; // вкл/выкл :) через кнопку start
int mode = 0; // текущий режим управления
int modesCount = 2;

unsigned int counter = 0; // счетчик для прерывания

unsigned int timer; // пригодится для подсчета времени выполнения операций



//=====================================
//  PS2 config
//=====================================
PS2X ps2x; // create PS2 Controller Class
int ps2error = 0; 
byte ps2type = 0;
byte ps2vibrate = 0;


//  начальные значения
const int coxaLeight = 33; // длина плеча в мм
const int femurLeight = 40; // 
const int tibiaLeight = 61; //
const int wsCenter = 90;

//  упраление одной ногой
int curLimb = 0; // номер ноги, который будем сейчас управлять
int controlMode = 0; // номер текущего типа управления одной ногой
const int controlModesNum = 3; // сколько всего у нас есть режимов управления ногой



//  для последовательности шагов
int stepHeight = 40; // высота шага
int stepLeight = 30; // радиус рабочей зоны или половина длины шага
int curSegment = 0; // текщий сегмент движения
const int stepSetup[6][7] = { 
  {
    3, 1, 3, 1, 3, 1, 4                                                                                                                                                                                                       }
  , {
    5, 0, 1, 2, 3, 4, 6                                                                                                                                                                                                       }
  , {
    5, 1, 3, 3, 1, 5, 6                                                                                                                                                                                                       }  
  , {
    5, 2, 5, 2, 5, 2, 6                                                                                                                                                                                                       }  
  , {
    11, 1, 3, 5, 7, 9, 12                                                                                                                                                                                                       } 
  , {
    11, 3, 7, 9, 5, 1, 12                                                                                                                                                                                                       } 
}; // для последовательностей шагов


const int giatHeight = 10; // клиренс
float dGiatHeight = 0;  //  высота походки


int giatNum = 0;
int numOfGiats = 6;
int curStep[6];

boolean walk = true; //  показывает, идем мы или стоим сейчас
int legsToSet = 6; // сколько ног ставить на место
boolean legsOnStartPos = true;
boolean stepConfigured = false;

float moveDirectionX = 0, moveDirectionY = 0; // вектор желаемого движения конечности (а в будущем и тела)
float mVectLeight = 0;  // а это длина этого вектора


float turnDirection = 0; // поворот - делается левым джойстиком
float turnAngle; //  угол поворота, аналог stepLeight, только для поворота
float segmentAng;
int turnSide;


float dx[6], dy[6], dz[6];
float biggestStep = 0;
float turnStepLeight[6];

const float limbNormAngle[6] = {
  135, 90, 45, -135, -90, -45};
float limbNormAngleSin[6], limbNormAngleCos[6];  // понадобится для преобразования координат

float coxaPosition[6][2] = {
  {
    -90, 52.5                                                                                                                          }
  ,
  {
    0, 62.5                                                                                                                          }
  ,
  {
    90, 52.5                                                                                                                          }
  ,
  {
    -90, -52.5                                                                                                                          }
  ,
  {
    0, -62.5                                                                                                                          }
  ,
  {
    90, -52.5                                                                                                                          }
};

float wsPosition[6][3]; // абсолютные координаты центра рабочей зоны; посчитаем дальше!!!

int angA[6], angB[6], angC[6];  //  углы для серв
float limbX[6], limbY[6], limbZ[6];  //  координаты положения ноги (относительные координаты)

int targetA, targetB, targetC;
float limbStepLeight[6]; // длина шага для каждой ноги
int segmentsInMove;

float stepVectorX[6];
float stepVectorY[6];
float longestCenterWSdist = 0;
float temp; //  пригодится 

float absX[6], absY[6];

boolean turboMode = false;




void setup(){

  SetupTimer (); // устанавливаем таймер; нам нужно, чтобы он выдавал нам примерно 20 мс задержки

  Serial.begin(9600);

  //hexOnProcedure ();

  pinMode(greenLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);

  //  конфигурим джойстик
  //  setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  ps2error = ps2x.config_gamepad(13,11,10,12, true, true);   

  if(ps2error == 0){
    tone(tonePin, 6000, 50);
    delay(50);
    tone(tonePin, 10000, 50);
  }

  //  Начальные установки для всякого

  for (int i = 0; i < 6; i++) {
    curStep[i] = stepSetup[giatNum][i];

    limbX[i] = wsCenter;
    limbY[i] = 0;
    limbZ[i] = giatHeight;

    limbNormAngleSin[i] = sin(radians(limbNormAngle[i]));
    limbNormAngleCos[i] = cos(radians(limbNormAngle[i]));

    // заодно здесь же считаем положения рабочих зон и расстояние до них
    wsPosition[i][0] = coxaPosition[i][0] + limbNormAngleCos[i] * wsCenter; //
    wsPosition[i][1] = coxaPosition[i][1] + limbNormAngleSin[i] * wsCenter; //
    float wsDist = sqrt(wsPosition[i][0] * wsPosition[i][0] + wsPosition[i][1] * wsPosition[i][1]);

    // находим самое большое расстояние от центра тела до цента рабочей зоны
    if (wsDist > longestCenterWSdist) longestCenterWSdist = wsDist;
  }
}

void loop(){

  if ((ps2error == 1) || (ps2type == 2)) //skip loop if no controller found or Guitar Hero Controller
    return; 

  else { //DualShock Controller

    if (counter >= 5) {

      timer = -millis();

      counter = 0; // обнуляем счетчик

      ps2x.read_gamepad(false, ps2vibrate);          //read controller and set large motor to spin at 'vibrate' speed

      // переключение режимов кнопкой select
      if(ps2x.ButtonPressed(PSB_SELECT)) {
        mode ++;
        if (mode >= modesCount) mode = 0;

        Serial.print("Mode: ");
        Serial.println(mode);

        for (int i=0; i <= mode; i++) {
          tone(tonePin, 4000, 50);
          delay(100);
        }
        tone(tonePin, 2000, 50);
      }

      // включение и выключение кнопокй start
      if(ps2x.ButtonPressed(PSB_START)) {
        hexOn = !hexOn;

        if (hexOn) hexOnProcedure ();
        else hexOffProcedure ();
      }

      if (ps2x.ButtonPressed(PSB_L1))tone(tonePin, 2000, 50);

      if (ps2x.Button(PSB_L1)) {
        turboMode = true;
        digitalWrite(yellowLED, HIGH);
      } 
      else {
        turboMode = false;
        digitalWrite(yellowLED, LOW);
      }

      switch (mode) {
      case 0:  // последовательность шагов
        if(ps2x.ButtonPressed(PSB_GREEN)) {
          giatNum ++;
          if (giatNum >= numOfGiats)
            giatNum = 0;

          for (int i = 0; i < 6; i++) {
            curStep[i] = stepSetup[giatNum][i];
          }

          for (int i=0; i <= giatNum; i++) {
            tone(tonePin, 4000, 50);
            delay(100);
          }
        }

        // снимаем параметры с джойстика, смотрим, куда будем двигаться
        moveDirectionX = ps2x.Analog(PSS_RY) - 128;
        moveDirectionY = ps2x.Analog(PSS_LX) - 128;
        turnDirection = ps2x.Analog(PSS_RX) - 128;
        dGiatHeight = (ps2x.Analog(PSAB_R2) - ps2x.Analog(PSAB_R1)) * stepLeight / 256 ; //  в двном случае, 256 = 2 * 128

        if ((moveDirectionX != 0) || (moveDirectionY != 0) || (turnDirection != 0)) { 

          if (!walk) {
            walk = true;
            for (int i = 0; i < 6; i++) {
              curStep[i] = stepSetup[giatNum][i];
            }
            Serial.println("Go, Eva!");
          }

          doTurnSequence (giatNum, stepSetup[giatNum][6]);
        }
        else {

          if (walk) {  // только что перешли из режима хотьбы в стоячий режим
            walk = false;  // фиксируем положение стоя в этой переменной

            //  готовим инфу для того, чтобы расставить ноги по местам
            legsToSet = 5;
            for (int i=0; i < 6; i++) curStep[i] = 0;
            legsOnStartPos = false;  // пока еще не все ноги на местах
            stepConfigured = false;
          }

          setLegsToStartPosition ();
        }

        break;
      case 1:  // калибровка!!!! =================== плавное передвижение к указанным углам

        //  зеленым треугольником переключаем ногу, которой управляем
        if(ps2x.ButtonPressed(PSB_GREEN)) {

          //  затем переключаем ногу на другую
          curLimb ++;

          if (curLimb >= 6) curLimb = 0;

          tone(tonePin, 4000, 50);
        }

        targetA = map(ps2x.Analog(PSS_RX), 0, 255, 70, 110);
        targetB = map(ps2x.Analog(PSS_LY), 0, 255, 70, 110);
        targetC = map(ps2x.Analog(PSS_LX), 0, 255, 70, 110);

        //limbIK (targetC + 45, targetB - 45, targetA - 45, curLimb);  // тут хитрая магия с цифрами!

        Serial.print(targetA);
        Serial.print("\t");
        Serial.print(targetB);
        Serial.print("\t");
        Serial.print(targetC);
        Serial.print("\n");

        moveCoxa(curLimb, targetA);
        moveFemur(curLimb, targetB);
        moveTibia(curLimb, targetC);

        break;
      }

      timer += millis();
      //Serial.println (timer);
    }
  }
}

void setLegsToStartPosition (){

  if (!legsOnStartPos) {  // до тех пор, пока у нас еще есть ноги, которые нужно расставить по местам
    if (stepConfigured) {  //  если мы уже знаем, как будем двигаться, исполняем движение

      limbX[legsToSet] += dx[legsToSet];
      limbY[legsToSet] += dy[legsToSet];
      limbZ[legsToSet] += dz[legsToSet];

      limbIK(limbX[legsToSet], limbY[legsToSet], limbZ[legsToSet], legsToSet);        

      moveCoxa(legsToSet, angA[legsToSet]);
      moveFemur(legsToSet, angB[legsToSet]);
      moveTibia(legsToSet, angC[legsToSet]);

      curSegment --;

      if (curSegment <= 0) {
        curStep[legsToSet] ++;
        stepConfigured = false;

        if (curStep[legsToSet] >= 2) {
          curStep[legsToSet] = 0;
          legsToSet --;
          if (legsToSet < 0) {
            legsOnStartPos = true;
          }
        }
      }
    } 
    else {  //  если движение не сконфигурировано, нужно его посчитать
      curSegment = 8;  //  это скорость движения 

      if (curStep[legsToSet] == 0) {
        dx[legsToSet] = (wsCenter - limbX[legsToSet]) / curSegment;
        dy[legsToSet] = (0.0 - limbY[legsToSet]) / curSegment;
        dz[legsToSet] = ((giatHeight - stepHeight) - limbZ[legsToSet]) / curSegment; 
      } 
      else { 
        dx[legsToSet] = 0.0;
        dy[legsToSet] = 0.0;
        dz[legsToSet] = (giatHeight - limbZ[legsToSet]) / curSegment;
      }
      stepConfigured = true;
    }

  }
}

void doTurnSequence (int giatNum_, int stepsInGiat_) {

  //  если мы дошли до конца текущего шага, то начинаем конфигурировать следующий
  if (curSegment <= 0) {

    if (turnDirection > 0) turnSide = 1;  // выбираем направление поворота
    else if (turnDirection < 0) turnSide = -1;
    else turnSide = 0;

    //  задаем новое число сегментов в движении (оно же скорость движения)

    int turboMultipler = 1;
    if (turboMode) turboMultipler = 2;

    // измеряем длину вектора движения
    mVectLeight = sqrt(moveDirectionX * moveDirectionX + moveDirectionY * moveDirectionY);

    segmentsInMove = 24 - (int)constrain(mVectLeight / 12 + abs(turnDirection / 12), 0, 10) * turboMultipler; //  минимальная разница должна быть 4!!!
    curSegment = segmentsInMove; //  эту цифру будем декрементировать

    biggestStep = 0;

    // для каждого сегмента движения 
    for (int i=0; i < 6; i++) {

      //  выбираем длину шага для каждой конечности
      //  она пропорциональная векторной сумме вектора поворота и вектора движения
      //  абсолютные координаты
      stepVectorX[i] = (  wsPosition[i][1] / longestCenterWSdist) * turnDirection + moveDirectionX;
      stepVectorY[i] = (- wsPosition[i][0] / longestCenterWSdist) * turnDirection + moveDirectionY;

      //  абсолютный размер рабочей зоны
      limbStepLeight[i] = sqrt(stepVectorX[i] * stepVectorX[i] + stepVectorY[i] * stepVectorY[i]);

      //  выбираем самую большую рабочую зону
      if (limbStepLeight[i] > biggestStep) biggestStep = limbStepLeight[i];

      // параллельно переходим ко следующим движениям шага для каждой конечности
      curStep[i] ++;
      if (curStep[i] >= stepsInGiat_) 
        curStep[i] = 0;
    }

    //  угол поворота;  аналог stepLeight
    turnAngle = atan2(stepLeight * (turnDirection / biggestStep), longestCenterWSdist);

    // дельта угла, на который нужно поворачивать конечность - сегмент поворота
    segmentAng = 2 * turnAngle / (segmentsInMove * (stepsInGiat_ - 2)); 

    //  считаем каждый шаг
    for (int i = 0; i < 6; i++) {

      if (curStep[i] == 0) {  //  поднимаем ногу в середину рабочей зоны
        // вычисляем разницу между тем, где мы есть и тем, куда хотим попасть и сразу делим на число сегментов движения
        dx[i] = (wsCenter - limbX[i]) / curSegment;
        dy[i] = (0 - limbY[i]) / curSegment;
        dz[i] = ((giatHeight + dGiatHeight - stepHeight) - limbZ[i]) / curSegment;
      } 
      else if (curStep[i] == 1) {  //  опускаем ногу в начало шага   

        // пересчитываем координаты вектора шага: 
        if (limbStepLeight[i] != 0) {
          stepVectorX[i] = stepVectorX[i] * stepLeight / biggestStep;
          stepVectorY[i] = stepVectorY[i] * stepLeight / biggestStep;
        } 
        else {
          stepVectorX[i] = 0;
          stepVectorY[i] = 0;
        }

        dx[i] = (stepVectorX[i] * limbNormAngleCos[i] + stepVectorY[i] * limbNormAngleSin[i] - limbX[i] + wsCenter) / curSegment; // stepVectorX[i]
        dy[i] = (stepVectorX[i] * limbNormAngleSin[i] - stepVectorY[i] * limbNormAngleCos[i] - limbY[i]) / curSegment; // 
        dz[i] = (giatHeight + dGiatHeight - limbZ[i]) / curSegment;

      } 
      else if (curStep[i] > 1) {  //  двигаем ногу против вектора движения тела

        mVectLeight = sqrt(moveDirectionX * moveDirectionX + moveDirectionY * moveDirectionY);

        float devider;

        if ((mVectLeight != 0) && (segmentsInMove != 0) && ((stepsInGiat_ - 2) != 0))
          devider = 2 * (stepLeight / mVectLeight) / (segmentsInMove * (stepsInGiat_ - 2));
        else devider = 0;

        dx[i] = moveDirectionX * devider; 
        dy[i] = moveDirectionY * devider;  
        dz[i] = (giatHeight + dGiatHeight - limbZ[i]) / curSegment;

      }
    }
  }

  for (int i = 0; i < 6; i++) {
    if (curStep[i] > 1) {

      //  переводим локальные координаты положения конечности в абсолютные координаты
      absX[i] = limbX[i] * limbNormAngleCos[i] + limbY[i] * limbNormAngleSin[i] + coxaPosition[i][0];
      absY[i] = limbX[i] * limbNormAngleSin[i] - limbY[i] * limbNormAngleCos[i] + coxaPosition[i][1];

      if (turnSide != 0) { //  поворачиваем конечности, стоящие на земле на определенный угол в абсолютных коорд
        float turnCos = cos(segmentAng); // - turnSide * 
        float turnSin = sin(segmentAng); // - turnSide * 

        float tempX = absX[i] * turnCos - absY[i] * turnSin;
        float tempY = absX[i] * turnSin + absY[i] * turnCos;

        absX[i] = tempX;
        absY[i] = tempY;
      }

      absX[i] -= dx[i];
      absY[i] -= dy[i];

      limbX[i] = (absX[i] - coxaPosition[i][0]) * limbNormAngleCos[i] + (absY[i] - coxaPosition[i][1]) * limbNormAngleSin[i];
      limbY[i] = (absX[i] - coxaPosition[i][0]) * limbNormAngleSin[i] - (absY[i] - coxaPosition[i][1]) * limbNormAngleCos[i];
      limbZ[i] += dz[i];

    } 
    else {

      limbX[i] += dx[i];
      limbY[i] += dy[i];
      limbZ[i] += dz[i];
    }
    limbIK(limbX[i], limbY[i], limbZ[i], i);        

    moveCoxa(i, angA[i]);
    moveFemur(i, angB[i]);
    moveTibia(i, angC[i]);

  }
  curSegment --;
}



// обратная кинематика для ног
void limbIK (float x_, float y_, float z_, int i_) {  //  получаем на вход координаты и номер ноги, для которой считаем параметры
  float op = sqrt(x_ * x_ + y_ * y_);
  float mp = op - coxaLeight;
  float bp = sqrt(mp * mp + z_ * z_);
  float acosB = (tibiaLeight * tibiaLeight + bp * bp - femurLeight * femurLeight) / (2 * tibiaLeight * bp);

  angA[i_] = degrees(HALF_PI + atan2(y_ , x_));
  if ((i_ >=0) && (i_ <=2)) {
    angB[i_] = degrees(HALF_PI - acos(acosB) +  acos(mp / bp) * sign(z_));
    angC[i_] = degrees(acos((tibiaLeight * tibiaLeight + femurLeight * femurLeight - bp * bp) / (2 * tibiaLeight * femurLeight)));
  } 
  else { // для левых ног надо крутить femur и tibia сервы в обратную сторону, т.к. они для красоты привинчены в другую сторону!!!
    angB[i_] = 180 - degrees(HALF_PI - acos(acosB) +  acos(mp / bp) * sign(z_));
    angC[i_] = 180 - degrees(acos((tibiaLeight * tibiaLeight + femurLeight * femurLeight - bp * bp) / (2 * tibiaLeight * femurLeight)));
  }
}

// определяем знак переменной
int sign (float a_) {
  int b;
  if (a_ >= 0) {
    b = 1;
  }
  else b = -1;
  return b;
}

void SetupTimer(){
  // настраиваем аппаратный таймер 4 для ATmega2560 в Arduino MEGA

  // снимаем с таймера ШИМ и прочие функции
  TCCR4A = 0;

  // устанавливаем делитель на 1
  TCCR4B = 0<<CS42 | 0<<CS41 | 1<<CS40; 

  // разрешаем прерывание по переполнению таймера 4
  TIMSK4 = 1<<TOIE4;
}


ISR(TIMER4_OVF_vect) {
  // вектор обработки прерывания по переполнению таймера 5

  // инкрементируем счетчик - нам нужно набрать 5, чтобы получить задержку примерно в 20 мс.
  counter ++;
  //Serial.println(counter);
}

void hexOnProcedure () {

  digitalWrite(greenLED, HIGH);
  Serial.println("Turning Eva on");

  // аттачим сервы и задаем начальные значения углов

  for (int i = 0; i < 6; i++) {
    coxaServo[i].attach(coxaPin[i]);
    femurServo[i].attach(femurPin[i]);
    tibiaServo[i].attach(tibiaPin[i]);

    limbIK(limbX[i], limbY[i], limbZ[i], i);

    moveCoxa(i, angA[i]);
    moveFemur(i, angB[i]);
    moveTibia(i, angC[i]);

    delay (300);
  }

  Serial.println("Eva is ON");


  tone(tonePin, 2000, 50);
  delay(50);
  tone(tonePin, 4000, 50);
}


void hexOffProcedure () {

  digitalWrite(greenLED, LOW);

  // детачим сервы
  for (int i = 0; i < 6; i++) {
    coxaServo[i].detach();
    femurServo[i].detach();
    tibiaServo[i].detach();
  } 

  Serial.println("Eva is OFF");

  tone(tonePin, 4000, 150);
  delay(150);
  tone(tonePin, 2000, 50);
  delay(100);
  tone(tonePin, 2000, 50);
  delay(100);
}


void moveCoxa (int coxaNum_, int pos_) {
  if (pos_ > 170) pos_ = 160;
  if (pos_ < 10) pos_ = 20;
  coxaServo[coxaNum_].write(pos_ + coxaCorr[coxaNum_]);
}

void moveFemur (int femurNum_, int pos_) {
  if (pos_ > 170) pos_ = 170;
  if (pos_ < 10) pos_ = 10;
  femurServo[femurNum_].write(pos_ + femurCorr[femurNum_]);
}

void moveTibia (int tibiaNum_, int pos_) {
  if (pos_ > 170) pos_ = 170;
  if (pos_ < 10) pos_ = 10;
  tibiaServo[tibiaNum_].write(pos_ + tibiaCorr[tibiaNum_]);
}
