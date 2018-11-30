#include <Arduino.h>
#include <TimerOne.h>

#include <TimerOne.h>

#define TIME_INTR 200000
#define SJW 4
#define TSEG1 8
#define TSEG2 15
//pins definition
#define RX_PIN 2 // tem que ser 2 ou 3
#define TX_PIN 4  //verifiacar se poder
#define SSYNC_PIN 9
#define HSYNC_PIN 10
#define TQ_PIN 11
#define STATE0_PIN 12
#define STATE1_PIN 13
#define IDLE_PIN 3

#define LENGTHFRAME 93

int i = 0;
enum StatesBTL {sync, phase1, phase2};
StatesBTL statesBTL;

enum ToggleInput {hardSync, softSync, time_quantum};
ToggleInput toggleInput;

enum StatesBitStuff {START, READ0, READ1, BIT_STUFF};
StatesBitStuff statesBitStuff;

enum StatesDecoder {START_D, ID, RTR_SRR, IDE, ID_EXTEND, RTR_EXTEND, R1, R0, DLC, DATA, CRC,
                    CRC_DELIMITER, ACK_SLOT, ACK_DELIMITER, OEF, IFS, ERROR_FLAG, ERROR_FLAG2, ERROR_DELIMITER,
                    OVERLOAD_FLAG, OVERLOAD_FLAG2, OVERLOAD_DELIMITER
                   };
StatesDecoder statesDecoder;

//enum StatesError {ERROR_FLAG, ERROR_FLAG2, ERROR_DELIMITER};
//StatesError statesError;

//enum StatesOverload {OVERLOAD_FLAG, OVERLOAD_FLAG2, OVERLOAD_DELIMITER};
//StatesOverload statesOverload;

uint8_t FE = 0, idle = 0, WP = 0, minimum = 0, sample_point, TQChange = 0, flag = 1;
volatile uint8_t BTLCount = 0; // use volatile for shared variables
byte hsValue = 0, tqValue = 0 , ssValue = 0, states = 0, STATE0 = 0, STATE1 = 0;


//variaveis para o bitstuff
uint8_t bit_stuff = 0, bit_stuff_value, bit_stuff_error = 0, bit_stuff_enable = 0;

//variaveis para o bitstuff2
uint8_t bit_stuff2 = 0, bit_stuff_value2, bit_stuff_error2 = 0, bit_stuff_enable2 = 0;

//variaveis para o decoder
uint8_t count = 0, over_count = 0, er_count = 0, data_length;
uint8_t start_d, rtr_srr, ide, r0, rtr_extend, r1, crc_delimiter, ack_delimiter, ack_slot;
uint8_t id[11], id_extend[18], dlc[4], data[64], crc[15], oef[7], ifs[3], error_enable, overload_enable, busWrite_value, decoder_enable=0;


//variaveis para o encoder
uint8_t arbitration=0,newBitValue=0, newWriteData=0, bitReaded;
//Variaveis para o crc
uint8_t crc_check[15], enable_crc, transmission_enable;

void busWrite(uint8_t value){
  newWriteData = 1;
  busWrite_value = value;
  Serial.print(value);
}

//Calcula o tamanho do campo de dados
uint8_t numberOfData() {

  uint8_t soma = 0;
  uint8_t pot=1;


  for(int i = 0; i<4; i++ ){
    pot=1;
    for(int j=0;j < i; j++){
      pot = 2*pot;
    }
    soma += dlc[3-i] * pot;
  }
  
  if (soma > 8) {
    soma = 8;
    return soma *8;
  }
  else {
    return soma * 8;
  }
}


void crc_calculator(uint8_t nextBit){
  uint8_t crcNext;
  crcNext = nextBit^crc_check[0];
  for(int j = 0; j<14; j++){
    crc_check[j]= crc_check[j+1];
  }
  crc_check[14] = 0;
  if(crcNext){
    crc_check[14] = crc_check[14] ^ 1;
    crc_check[13] = crc_check[13] ^ 0;
    crc_check[12] = crc_check[12] ^ 0;
    crc_check[11] = crc_check[11] ^ 1;
    crc_check[10] = crc_check[10] ^ 1;
    crc_check[9] = crc_check[9] ^ 0;
    crc_check[8] = crc_check[8] ^ 0;
    crc_check[7] = crc_check[7] ^ 1;
    crc_check[6] = crc_check[6] ^ 1;
    crc_check[5] = crc_check[5] ^ 0;
    crc_check[4] = crc_check[4] ^ 1;
    crc_check[3] = crc_check[3] ^ 0;
    crc_check[2] = crc_check[2] ^ 0;
    crc_check[1] = crc_check[1] ^ 0;
    crc_check[0] = crc_check[0] ^ 1;
  }

}


void resetCRC(){
  for (int j = 0; j < 15; j++){
    crc_check[j] = 0;
  }
}

void toggle(ToggleInput input) {

  switch (input) {
    case hardSync:
      hsValue = !hsValue;
      digitalWrite(HSYNC_PIN, hsValue);
      break;
    case softSync:
      ssValue = !ssValue;
      digitalWrite(SSYNC_PIN, ssValue);
      //      Serial.print("SSYNC_PIN ");
      break;
    case time_quantum:
      tqValue = !tqValue;
      digitalWrite(TQ_PIN, tqValue);
      break;
  }
}

void fallingEdgeDetector() {
  FE = 1;
}

void idleDetector() {
  idle = digitalRead(IDLE_PIN);
}

void updateTQ(void) {
  /*                            //DEBUG CODE
    Serial.print("Estado: ");
    Serial.print(statesBTL);
    Serial.print(" , FE: ");
    Serial.print(FE);
    Serial.print(" , idle: ");
    Serial.print(idle);
    Serial.print(" , BTLCount: ");
    Serial.println(BTLCount); */

  BTLCount < TSEG2 ? BTLCount ++ : BTLCount = 0;
  toggle(time_quantum);
  sample_point = 0;
  WP = 0;


  if (BTLCount == 2 && flag) { //DEBUG CODE
    FE = 1;
    flag = 0;
  }
  TQChange = 1;

}

void btlLogic() {
  if (TQChange) {
    switch (statesBTL) {
      case sync:
        WP = 1;
        /* output da entrega
        STATE0 = 0;
        STATE1 = 0;
        digitalWrite(STATE0_PIN, 0);
        digitalWrite(STATE1_PIN, 0);
        /*/

        if (BTLCount == 1) {
          statesBTL = phase1;
        }
        else if (BTLCount == 0 && FE) {
          FE = 0;
        }
        break;

      case phase1:
        /* output da entrega
        STATE0 = 0;
        STATE1 = 1;
        digitalWrite(STATE0_PIN, 0);
        digitalWrite(STATE1_PIN, 1);
        */
        WP = 0;
        
        if ( FE && idle) {
          toggle(hardSync);
          noInterrupts();
          BTLCount = 1;
          interrupts();
          FE = 0;
        }
        else if (BTLCount > TSEG1) {
          statesBTL = phase2;

        }
        else if (FE) {
          toggle(softSync);
          minimum = min(SJW, BTLCount);
          noInterrupts();
          BTLCount -= minimum;
          interrupts();
          if (BTLCount == 0) {
            noInterrupts();
            BTLCount++;
            interrupts();
          }
          FE = 0;
          //Serial.println("Entrou"); DEBUG CODE

        }
        break;
      case phase2:
        // output da entrega
        STATE0 = 1;
        STATE1 = 0;
        digitalWrite(STATE0_PIN, 1);
        digitalWrite(STATE1_PIN, 0);

        if (BTLCount == TSEG1 + 2){
           sample_point = 1;
           bitReaded = digitalRead(TX_PIN);
           newBitValue = 1;
        }

        if (BTLCount == 0) {
          statesBTL = sync;
        }
        else if ( FE && idle) {
          toggle(hardSync);
          noInterrupts();
          BTLCount = 1;
          interrupts();
          FE = 0;
          statesBTL = phase1;
        }
        else if (FE) {
          toggle(softSync);
          minimum = min(SJW, TSEG2 - BTLCount + 1); //+1 pq o BTLCount começa a contar do 0
          noInterrupts();
          BTLCount += minimum;
          interrupts();
          if ( BTLCount > TSEG2 ) { // pode ser quer a soma passe de 16
            BTLCount = 1;
            statesBTL = phase1;
          }
          FE = 0;
          break;
        }
    }

  }
  TQChange = 0;
}

//maquina de estado do bitstuff
void bitStuff(uint8_t bitValue) {
  static uint8_t count0;
  static uint8_t count1;
  if (bit_stuff_enable == 1) {
    switch (statesBitStuff) {
      case START:
        count0 = 0;
        count1 = 0;
        bit_stuff = 0;    //variavel responsavel por indicar a ocorrencia de um bit stuff

        if (bitValue == 0) {
          statesBitStuff = READ0;
          count0++;
        }
        else {
          statesBitStuff = READ1;
          count1++;
        }
        break;
      case READ0:
        if (bitValue == 0 && count0 < 6) {
          count0++;
        }
        else if (bitValue == 1 && count0 < 6) {
          statesBitStuff = READ1;
          count1++;
          count0 = 0;
        }
        else if (count0 == 6) {
          statesBitStuff = BIT_STUFF;
          bit_stuff = 1;                  //indica a ocorrencia de um bit stuff
          bit_stuff_value = !bitValue;
          if(decoder_enable && bitValue == 0){
            bit_stuff_error = 1;
          }
        }
        break;
      case READ1:
        if (bitValue == 0 && count1 < 6) {
          count1++;
        }
        else if (bitValue == 1 && count1 < 6) {
          statesBitStuff = READ0;
          count0++;
          count1 = 0;
        }
        else if (count1 == 6) {
          statesBitStuff = BIT_STUFF;
          bit_stuff = 1;                  //indica a ocorrencia de um bit stuff
          bit_stuff_value = !bitValue;
          if(decoder_enable && bitValue == 1){
            bit_stuff_error = 1;
          }
        }
        break;
        /*case Error_Frame:
          bit_stuff_error = 1;
          break;*/
      case BIT_STUFF:
        bit_stuff = 0;
        count0 = 0;
        count1 = 0;

        if (bitValue == 0) {
          statesBitStuff = READ0;
          count0++;
          count1 = 0;
        }
        else {
          statesBitStuff = READ1;
          count0 = 0;
          count1++;
        }
        break;
    }
  }
}

// void bitStuff2(uint8_t bitValue) {
  //   static uint8_t count0;
  //   static uint8_t count1;
  //   if (bit_stuff_enable2 == 1) {
  //     switch (statesBitStuff2) {
  //       case START:
  //         count0 = 0;
  //         count1 = 0;
  //         bit_stuff2 = 0;    //variavel responsavel por indicar a ocorrencia de um bit stuff

  //         if (bitValue == 0) {
  //           statesBitStuff2 = READ0;
  //           count0++;
  //         }
  //         else {
  //           statesBitStuff2 = READ1;
  //           count1++;
  //         }
  //         break;
  //       case READ0:
  //         if (bitValue == 0 && count0 < 5) {
  //           count0++;
  //         }
  //         else if (bitValue == 1 && count0 <=5) {
  //           statesBitStuff2 = READ1;
  //           count1 = 0;
  //           count1++;
  //           count0 = 0;
  //         }
  //         else if (count0 == 5) {
  //           statesBitStuff2 = BIT_STUFF;
  //           bit_stuff2 = 1;                  //indica a ocorrencia de um bit stuff
  //           bit_stuff_value2 = !bitValue;
  //         }
        
  //         else {
  //           statesBitStuff2 = ERROR_FRAME;
  //           bit_stuff_error2 = 1;
  //         }
  //         break;
  //       case READ1:
  //         if (bitValue == 1 && count1 < 6 ) {
  //           count1++;
  //         }
  //         else if (bitValue == 0 && count1 == 5) {
  //           statesBitStuff2 = BIT_STUFF;
  //           bit_stuff2 = 1;                //indica a ocorrencia de um bit stuff
  //           bit_stuff_value2 = !bitValue;
  //         }
  //         else if (bitValue == 0 && count1 < 6) {
  //           statesBitStuff2 = READ0;
  //           count0++;
  //           count1 = 0;
  //         }
  //         else {
  //           statesBitStuff2 = ERROR_FRAME;
  //           bit_stuff_error2 = 1;
  //         }
  //         break;
  //         /*case Error_Frame:
  //           bit_stuff_error = 1;
  //           break;*/
  //       case BIT_STUFF:
  //         bit_stuff2 = 0;
  //         count0 = 0;
  //         count1 = 0;

  //         if (bitValue == 0) {
  //           statesBitStuff2 = READ0;
  //           count0++;
  //           count1 = 0;
  //         }
  //         else {
  //           statesBitStuff2 = READ1;
  //           count0 = 0;
  //           count1++;
  //         }
  //         break;
  //     }
  //   }
// }

 



//implementacao da Maquina de estados do decoder
void decoderLogic(uint8_t bitValue) {

  switch (statesDecoder) {
    case START_D:
      Serial.print("Start: "); //print
      enable_crc = 1;
      resetCRC();

      if (bitValue == 1) {
        count = 0;  
        bit_stuff = 0;
        er_count = 0;
        bit_stuff_enable = 0;
      }
      else {
        Serial.println(start_d);  //print
        statesDecoder = ID;
        Serial.print("ID: "); //print
        bit_stuff_enable = 1;
        bitStuff(bitValue);
        start_d = bitValue;
        count = 0;
        

      }
      break;
    case ID:
      bitStuff(bitValue);
      
      if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      else {
        if (count < 10 && bit_stuff == 0) {
          id[count] = bitValue;
          Serial.print(id[count]); //print
          count++;
        }
        else if (count == 10 && bit_stuff == 0) {
          statesDecoder = RTR_SRR;
          id[count] = bitValue;
          Serial.println(id[count]); //print
          count++;
        }
      }
      break;
    case RTR_SRR:
      bitStuff(bitValue);
      Serial.print("RTR_SRR: "); //print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        statesDecoder = IDE;
        rtr_srr = bitValue;
        Serial.println(rtr_srr); //print
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case IDE:
      bitStuff(bitValue);
      Serial.print("IDE: "); //print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        ide = bitValue;
        Serial.println(ide); //print
        count = 0;

        if (ide == 1) {
          statesDecoder = ID_EXTEND;
          Serial.print("ID_EXTEND: "); //print
        }
        else {
          statesDecoder = R0;
        }
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case ID_EXTEND:
      bitStuff(bitValue);
       
      if (bit_stuff == 0 && bit_stuff_error == 0 && count < 17) {
        id_extend[count] = bitValue;
        Serial.print(id_extend[count]); //print
        count++;
      }
      else if (count == 17) {
        id_extend[count] = bitValue;
        Serial.println(id_extend[count]); //print
        statesDecoder = RTR_EXTEND;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case RTR_EXTEND:
      bitStuff(bitValue);
      Serial.print("RTR_EXTEND: "); // print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        rtr_extend = bitValue;
        Serial.println(rtr_extend); //print
        statesDecoder = R1;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case R1:
      bitStuff(bitValue);

      if (bit_stuff == 0 && bit_stuff_error == 0) {
        Serial.print("R1: ");
        Serial.println(r1);
        r1 = bitValue;
        statesDecoder = R0;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case R0:
      bitStuff(bitValue);

      if (bit_stuff == 0 && bit_stuff_error == 0) {
        r0 = bitValue;
        count = 0;
        statesDecoder = DLC;
        Serial.print("R0: ");
        Serial.println(r0);
        Serial.print("DLC: ");
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case DLC:
      bitStuff(bitValue);

      if (count < 3 && bit_stuff == 0 && bit_stuff_error == 0) {
        dlc[count] = bitValue;
        Serial.print(dlc[count]);
        count ++;
      }
      else if (count == 3 && bit_stuff == 0 && bit_stuff_error == 0) {
        dlc[count] = bitValue;
        Serial.println(dlc[count]);
        count ++;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }

      if (count == 4) {
        //calcula o datanho do campo de dados
        //        for(int i=0;i<4;i++){
        //          Serial.print("i: ");
        //          Serial.print(i);
        //          Serial.print("dlc: ");
        //          Serial.println(dlc[i]);
        //        }
        data_length = numberOfData();
        //        Serial.print("Tamanho dos dados: ");
        //        Serial.print(data_length);

        if ((/*count == 3 &&*/ ( (rtr_srr == 1 && ide == 0) || (ide == 1 && rtr_extend == 1) ) || data_length == 0) ) {
          statesDecoder = CRC;
          Serial.print("CRC: ");
          count = 0;
        }
        else {
          statesDecoder = DATA;
          Serial.print("DATA: ");
          count = 0;
        }
      }
      break;
    case DATA:
      bitStuff(bitValue);

      //Serial.println(data_length);
      if (count < data_length && bit_stuff == 0 && bit_stuff_error == 0) {
        data[count] = bitValue;
        Serial.print(data[count]);
      }
      else if (bit_stuff_error == 1 && count < data_length) {     //Verifica se ocorreu um erro enquanto esta lendo os dados
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }

      count ++;
      //Se o tamanho dos dados forem menor que 8 ignora os proximo bit ate mudar de estados
      if (count == data_length) {
        statesDecoder = CRC;
        Serial.println("");
        Serial.print("CRC: ");
        count = 0;
      }
      break;
    case CRC:
      enable_crc = 0;
      bitStuff(bitValue);

      //      Serial.println(count);
      if (count < 14 && bit_stuff == 0 && bit_stuff_error == 0) {
        crc[count] = bitValue;
        Serial.print(crc[count]);
        if(crc[count] != crc_check[count] ){
          //Serial.println("crc count: " + String(crc[count]) + " crc_check:" + String(crc_check[count]));
          statesDecoder = ERROR_FLAG;
          Serial.print("Error Flag: ");
      }
       
        count++;
      }
      else if (count == 14 && bit_stuff == 0 && bit_stuff_error == 0) {
        crc[count] = bitValue;
        Serial.println(crc[count]);
        statesDecoder = CRC_DELIMITER;
        if(crc[count] != crc_check[count] ){
          //Serial.println("crc count: " + String(crc[count]) + " crc_check:" + String(crc_check[count]));
          statesDecoder = ERROR_FLAG;
          Serial.print("Error Flag: ");
      }
        bit_stuff_enable = 0;         //desabilita a verificacao do bitStuff
        count = 0;
      }
      break;
    case CRC_DELIMITER:
      
      crc_delimiter = bitValue;
      Serial.print("CRC_delimiter: ");
      Serial.println(crc_delimiter);
      if (crc_delimiter == 1) {
        statesDecoder = ACK_SLOT;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case ACK_SLOT:
      ack_slot = bitValue;
       Serial.print("ACK_SLOT: ");
       Serial.println(ack_slot);
      statesDecoder = ACK_DELIMITER;
          
      //busWrite(0);

      break;
    case ACK_DELIMITER:
      ack_delimiter = bitValue;
      Serial.print("ACK_DELIMITER: ");
      Serial.println(ack_delimiter);
      if (ack_delimiter == 1) {
        statesDecoder = OEF;
        Serial.print("OEF: ");
        count = 0;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case OEF:

      if (bitValue == 1 && count < 6) {
        oef[count] = bitValue;
        Serial.print(oef[count]);
        count++;
      }
      else if (bitValue == 1 && count == 6) {
        oef[count] = bitValue;
        Serial.println(oef[count]);
        statesDecoder = IFS;
        Serial.print("IFS: ");
        count = 0;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
      }
      break;
    case IFS:
      if (count < 2 && bitValue == 1) {
        ifs[count] = bitValue;
        Serial.print(ifs[count]);
        count++;
      }
      if (count == 2 && bitValue == 1) {
        ifs[count] = bitValue;
        Serial.println(ifs[count]);
        statesDecoder = START_D;
        count = 0;
      }
      else if (bitValue == 0) {
        count = 0;
        statesDecoder = OVERLOAD_FLAG;
      }
      break;

    case ERROR_FLAG:
      if ( er_count < 5) {
        er_count++;
        busWrite(0);
      }
      else if ( er_count == 5) {
                   busWrite(0);
        statesDecoder = ERROR_FLAG2;
        Serial.print("Error Flag2: ");
      }
      break;
    case ERROR_FLAG2:

      if (WP == 1 && digitalRead(RX_PIN) == 1) {
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      else if (WP == 1 && digitalRead(RX_PIN) == 0 && er_count < 11) {
                   busWrite(0);
        er_count++;
      }
      else if (er_count == 11) {
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      break;
    case ERROR_DELIMITER:

      if (WP == 1 && count < 7) {
         
        busWrite(1);
        count++;
      }
      else if (WP == 1 && count == 7) {
         
        busWrite(1);
        statesDecoder = IFS;
      }
      break;

    case OVERLOAD_FLAG:
      if (WP == 1 && over_count < 5) {
        over_count++;

             
         busWrite(0);
      }
      else if (WP == 1 && over_count == 5) {
         
         busWrite(0);
        statesDecoder = OVERLOAD_FLAG2;
      }
      break;
    case OVERLOAD_FLAG2:

      if (WP == 1 && digitalRead(RX_PIN) == 1) {
        statesDecoder = OVERLOAD_DELIMITER;
        count = 0;
      }
      else if (WP == 1 && digitalRead(RX_PIN) == 0 && er_count < 11) {
         
         busWrite(0);
        er_count++;
      }
      else if (er_count == 11) {
         
         busWrite(0);
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      break;
    case OVERLOAD_DELIMITER:

      if (WP == 1 && count < 7) {
         
        busWrite(1);

        count++;
      }
      else if (WP == 1 && count == 7) {
         
        busWrite(1);

        statesDecoder = IFS;
      }
      break;
  }
  if(enable_crc == 1 && !bit_stuff){
    crc_calculator(bitValue);
  }
}

void encoderLogic(uint8_t bitValue) {
  switch (statesDecoder) {
    case START_D:

      enable_crc = 1;
      resetCRC();
      //Serial.println("Entrou");
      arbitration = 1; // habilita arbitração

      
      count = 0;  
      bit_stuff = 0;
      er_count = 0;
      bit_stuff_enable = 0;
      
      busWrite(0);

      statesDecoder = ID;
      bit_stuff_enable = 1;
      bitStuff(bitValue);
      start_d = bitValue;
      count = 0;
      Serial.print(" ");
      
      break;
    case ID:
      bitStuff(bitValue);
      if (bit_stuff) {
        busWrite(bit_stuff_value); //VERIFICAR COMO RECOMEÇAR DO MESMO BITVALUE
      }
      else {
        if (count < 10 && bit_stuff == 0) {
          busWrite(bitValue);
          count++;
        }
        else if (count == 10 && bit_stuff == 0) {
          statesDecoder = RTR_SRR;
          busWrite(bitValue);
          Serial.print(" ");
          count++;
        }
      }
      
      break;
    case RTR_SRR:
      bitStuff(bitValue);
      arbitration = 0; //desabilita arbitração
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else{
        rtr_srr = bitValue;
        statesDecoder = IDE;
        busWrite(bitValue);
      }
      break;
    case IDE:
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else{
        ide = bitValue;
        busWrite(bitValue);
        count = 0;

        if (ide == 1) {
          statesDecoder = ID_EXTEND;
        }
        else {
          statesDecoder = R0;
        }
      }
      break;
    case ID_EXTEND:
      bitStuff(bitValue);
      arbitration = 1;
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else if (count < 17) {
        busWrite(bitValue);
        count++;
      }
      else if (count == 17) {
        busWrite(bitValue);
        statesDecoder = RTR_EXTEND;
        Serial.print(" ");
      }
      break;
    case RTR_EXTEND:
      arbitration = 0;
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else{
        busWrite(bitValue);
        rtr_extend = bitValue;
        statesDecoder = R1;
      }
      break;
    case R1:
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else{
        r1 = bitValue;
        busWrite(bitValue);
        statesDecoder = R0;
      }
      break;
    case R0:
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else{
        busWrite(bitValue);
        count = 0;
        statesDecoder = DLC;
        Serial.print(" ");
      }
      break;
    case DLC:
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      else if (count <= 3) {
        dlc[count] = bitValue;
        busWrite(bitValue);
        count ++;
      }

      if (count == 4) {
        //calcula o datanho do campo de dados
        //        for(int i=0;i<4;i++){
        //          Serial.print("i: ");
        //          Serial.print(i);
        //          Serial.print("dlc: ");
        //          Serial.println(dlc[i]);
        //        }
        data_length = numberOfData();
        //Serial.print("Tamanho dos dados: ");
        //Serial.print(data_length);

        if (/*count == 3 &&*/ ( (rtr_srr == 1 && ide == 0) || (ide == 1 && rtr_extend == 1) ) || data_length == 0 ) {
          statesDecoder = CRC;
          count = 0;
          Serial.print(" ");
        }
        else {
          statesDecoder = DATA;
          count = 0;
          Serial.print(" ");
        }
      }
      break;
    case DATA:
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
      }
      if (count < data_length && bit_stuff == 0 && bit_stuff_error == 0) {
        busWrite(bitValue);
        count ++;
      }
      else if (bit_stuff_error == 1 && count < data_length) {     //Verifica se ocorreu um erro enquanto esta lendo os dados
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: ");
        count = 0;
      }
      

      //Se o tamanho dos dados forem menor que 8 ignora os proximo bit ate mudar de estados
      if (count == data_length) {
        statesDecoder = CRC;
        count = 0;
        Serial.print(" ");
      }
      break;
    case CRC:
      enable_crc = 0;
      bitStuff(bitValue);
      if(bit_stuff){
        busWrite(bit_stuff_value);
        Serial.print("*");
        bitStuff(bit_stuff_value);  //o bit de bit stuff tambem conta para o novo bitstuff
        i--;//se tiver realizado o bitstuff o encoder nao deve avançar no frame
      }
      else if (count < 14) {
        busWrite(bitValue);
        count++;
      }
      else if (count == 14) {
        busWrite(bitValue);
        statesDecoder = CRC_DELIMITER;
        bit_stuff_enable = 0;         //desabilita a verificacao do bitStuff
        count = 0;
        Serial.print(" ");
      }
      break;
    case CRC_DELIMITER:
      busWrite(crc_delimiter);
      statesDecoder = ACK_SLOT;
      break;
    case ACK_SLOT:
      busWrite(bitValue);
      statesDecoder = ACK_DELIMITER;
      break;
    case ACK_DELIMITER:
      busWrite(ack_delimiter);
      statesDecoder = OEF;
      count = 0;
      Serial.print(" ");
      break;
    case OEF:
      if (bitValue == 1 && count < 6) {
        busWrite(bitValue);
        count++;
      }
      else if (bitValue == 1 && count == 6) {
        busWrite(bitValue);
        statesDecoder = IFS;
        Serial.print(" ");
        count = 0;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("Error Flag: "); 
      }
      break;
    case IFS:
      if (count < 2 && bitValue == 1) {
        busWrite(ifs[count]);
        count++;
      }
      if (count == 2 && bitValue == 1) {
        busWrite(ifs[count]);
        idle = 1;
        statesDecoder = START_D;
        count = 0;
        Serial.print(" ");
      }
      else if (bitValue == 0) {
        count = 0;
        statesDecoder = OVERLOAD_FLAG;
      }
      Serial.println("");
      break;

    case ERROR_FLAG:
      if (WP == 1 && er_count < 5) {
        er_count++;
        busWrite(0);
      }
      else if (WP == 1 && er_count == 5) {
        busWrite(0);
        statesDecoder = ERROR_FLAG2;
        }
      break;
    case ERROR_FLAG2:

      if (WP == 1 && digitalRead(RX_PIN) == 1) {
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      else if (WP == 1 && digitalRead(RX_PIN) == 0 && er_count < 11) {
        busWrite(0);
        er_count++;
      }
      else if (er_count == 11) {
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      break;
    case ERROR_DELIMITER:

      if (count < 7) {
         
        busWrite(1);
        count++;
      }
      else if ( count == 7) {
         
        busWrite(1);
        statesDecoder = IFS;
      }
      break;

    case OVERLOAD_FLAG:
      if (WP == 1 && over_count < 5) {
        over_count++;           
         busWrite(0);
      }
      else if (WP == 1 && over_count == 5) {
         busWrite(0);
        statesDecoder = OVERLOAD_FLAG2;
      }
      break;
    case OVERLOAD_FLAG2:

      if (WP == 1 && digitalRead(RX_PIN) == 1) {
        statesDecoder = OVERLOAD_DELIMITER;
        count = 0;
      }
      else if (WP == 1 && digitalRead(RX_PIN) == 0 && er_count < 11) {
         
         busWrite(0);
        er_count++;
      }
      else if (er_count == 11) {
        busWrite(0);
        statesDecoder = ERROR_DELIMITER;
        count = 0;
      }
      break;
    case OVERLOAD_DELIMITER:

      if (WP == 1 && count < 7) { 
        busWrite(1);

        count++;
      }
      else if (WP == 1 && count == 7) {
         
        busWrite(1);

        statesDecoder = IFS;
      }
      break;
  }
  if(enable_crc == 1 && !bit_stuff){
    crc_calculator(bitValue);
  }
}




void setup(void)
{
  Timer1.initialize(TIME_INTR);
  Timer1.attachInterrupt(updateTQ);

  attachInterrupt(digitalPinToInterrupt(RX_PIN), fallingEdgeDetector, FALLING);
  //  attachInterrupt(digitalPinToInterrupt(IDLE_PIN), idleDetector, RISING);

  pinMode(HSYNC_PIN, OUTPUT);
  pinMode(STATE0_PIN, OUTPUT);
  pinMode(STATE1_PIN, OUTPUT);
  pinMode(TQ_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(IDLE_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  Serial.begin(9600);
}



void loop(void){
//  uint8_t value =0;     //teste
//  if(newBitValue){
//    decoderLogic(bitReaded);
//    newBitValue = 0;
//  }
//  if(idle || transmission_enable){
//    idle = 0;
//    encoderLogic(value);
//  }
//
//  if(newWriteData && WP){
//    digitalWrite(TX_PIN, busWrite_value);
//    newWriteData = 0;
//    if(arbitration && busWrite_value != digitalRead(RX_PIN)){
//      transmission_enable = 0;
//      arbitration = 0;
//    }
//    else if (busWrite_value != digitalRead(RX_PIN)){
//      statesDecoder = ERROR_FLAG;
//    }
//  }

  
  //Teste frame com melhor caso
  uint8_t can_stand[LENGTHFRAME] = {0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,0,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,1,0,0,1,0,0,1,0,1,1,1,0,1,0,1,1,1,1,1,1,1,1};
  //uint8_t can_stand[LENGTHFRAME] = {0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,1,1,1,1,1,1};
  if (i < LENGTHFRAME) {
    decoderLogic(can_stand[i++]);
//    crc_calculator(can_stand[i]);
//
//    //    decoderLogic(can_extend[i]);
  }
  if(i == LENGTHFRAME){
    Serial.println("");
    Serial.print("crc_check: ");
    for(int k=0; k < 15;k++){
      Serial.print(crc_check[k]);
    }
    i++;
  }
  
  // i++;
//  for(int i=0; i < 83;i++){
//     crc_calculator(can_stand[i]);
//  }
//

//  Serial.println();

//  resetCRC();
  
/*
  //Serial.println("Soma: " + String(soma));
    uint8_t teste[12] = {0,0,0,0,0,1,1,0,0,1,0};
    statesBitStuff = 1;        Teste da Maquina de estados do bit stuff
    for(int i=0;i<12; i++){
    bitStuff(teste[i]);

    Serial.print("frame: ");
    Serial.print(teste[i]);
    Serial.print(", bitValue: ");
    Serial.print(bit_stuff_value);
    Serial.print(", Error: ");
    Serial.print(bit_stuff_error);
    Serial.print(", count1: ");
    Serial.print(count1);
    Serial.print(", count0: ");
    Serial.print(count0);
    Serial.print(", estados: ");
    Serial.print(statesBitStuff);
    Serial.print(", bit_stuff: ");
    Serial.println(bit_stuff);


    }
    /*
    idle = digitalRead(IDLE_PIN);

    Serial.print(hsValue);
    Serial.print(" ");

    Serial.print(ssValue + 2);
    Serial.print(" ");


    Serial.print(tqValue + 4);
    Serial.print(" ");

    Serial.print(STATE0 + 8);
    Serial.print(" ");

    Serial.print(sample_point + 10);
    Serial.print(" ");

    Serial.print(WP + 12);
    Serial.print(" ");

    Serial.println(STATE1 + 6);



    btlLogic();*/
}
