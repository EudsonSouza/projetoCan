#include <Arduino.h>
#include <TimerOne.h>

#include <TimerOne.h>

#define TIME_INTR 5000
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
StatesBitStuff statesBitStuff, statesBitStuff_E;

enum StatesDecoder {START_D, ID, RTR_SRR, IDE, ID_EXTEND, RTR_EXTEND, R1, R0, DLC, DATA, CRC,
                    CRC_DELIMITER, ACK_SLOT, ACK_DELIMITER, ENDOF, IFS, ERROR_FLAG, ERROR_FLAG2, ERROR_DELIMITER,
                    OVERLOAD_FLAG, OVERLOAD_FLAG2, OVERLOAD_DELIMITER
                   };
StatesDecoder statesDecoder, statesEncoder;

//enum StatesError {ERROR_FLAG, ERROR_FLAG2, ERROR_DELIMITER};
//StatesError statesError;

//enum StatesOverload {OVERLOAD_FLAG, OVERLOAD_FLAG2, OVERLOAD_DELIMITER};
//StatesOverload statesOverload;

uint8_t FE = 0, idle = 1, WP = 0, minimum = 0, sample_point, TQChange = 0, flag = 1;
volatile uint8_t BTLCount = 0; // use volatile for shared variables
byte hsValue = 0, tqValue = 0 , ssValue = 0, states = 0, STATE0 = 0, STATE1 = 0;


//variaveis para o bitstuff
uint8_t bit_stuff = 0, bit_stuff_value, bit_stuff_error = 0, bit_stuff_enable = 0, bit_stuff_e;
uint8_t count0;
uint8_t count1;

//variaveis para o bitstuff
uint8_t bit_stuff_E = 0, bit_stuff_value_E, bit_stuff_error_E = 0, bit_stuff_enable_E = 0;
uint8_t count0_E;
uint8_t count1_E;

//variaveis para o decoder
uint8_t count = 0, over_count = 0, er_count = 0, data_length;
uint8_t start_d, rtr_srr, ide, r0, rtr_extend, r1, crc_delimiter, ack_delimiter, ack_slot;
uint8_t id[11], id_extend[18], dlc[4], data[64], crc[15], endof[7], ifs[3], error_enable, overload_enable, busWrite_value, decoder_enable=0;


//variaveis para o encoder
uint8_t count_e = 0, data_length_e;
uint8_t rtr_srr_e, ide_e, rtr_extend_e, dlc_e[4];
uint8_t busWriteValue;
//Variaveis para o crc

uint8_t crc_check[15], enable_crc, encoder_enable, crc_error = 0;

// Variaveis de integração BTL - EncoderDecoder
uint8_t arbitration=0,newBitValue=0, newWriteDataFlag=0, bitReaded, rValue, newBitReaded, busReadValue;
uint8_t sample_pointFlag, WPFlag, diff_rx_tx;

//Variaveis Testte
uint8_t can_stand_D[] = {0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,1,1,1,1,1,1};
//uint8_t can_stand_E[] = {0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,1,1,1,1,1,1};

int j = 0;
void busWrite(uint8_t value){
  newBitValue = 1;
  busWriteValue = value;
  Serial.print(value);
}

void readAndWriteBus(){
  
  if (newBitValue && WP){
    //digitalWrite(TX_PIN, busWriteValue);
    busWriteValue = can_stand_D[j++];
    newBitValue = 0;
    WP = 0;
    WPFlag = 1;
  }
  else if (WP){
    digitalWrite(TX_PIN, 1);
    WP = 0;
  }
  if(sample_point){
    //busReadValue = digitalRead(RX_PIN);
    if (i<sizeof(can_stand_D)/sizeof(can_stand_D[0])) {////////////////////
      busReadValue = can_stand_D[i++];////////////////////
    }
    else{
      busReadValue = 1;
    }
    sample_point = 0;
    sample_pointFlag = 1;
    if(busReadValue == busWriteValue){
      diff_rx_tx = 0;
    }
    else{
      diff_rx_tx = 1;
    }
  
  }
}
  


//Calcula o tamanho do campo de dados
uint8_t numberOfData(boolean isDecoder) {

  uint8_t soma = 0;
  uint8_t pot=1;


  for(int i = 0; i<4; i++ ){
    pot=1;
    for(int j=0;j < i; j++){
      pot = 2*pot;
    }
    if(isDecoder){
      soma += dlc[3-i] * pot;
    }
    else{
      soma += dlc_e[3-i] * pot;
    }
    
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

void updateTQ() {
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
  if (bit_stuff_enable == 1) {
   // Serial.println(String(statesBitStuff) + " bit_stuff " + String(bit_stuff) + " count0 = " + String(count0) + " count1 = " + String(count1) );
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
        if (bitValue == 0 && count0 < 5) {
          count0++;
        }
        else if (bitValue == 1 && count0 < 5) {
          statesBitStuff = READ1;
          count1 = 0;
          count1++;
          count0 = 0;
        }
        else if (count0 == 5) {
          statesBitStuff = BIT_STUFF;
          bit_stuff = 1;                  //indica a ocorrencia de um bit stuff
          bit_stuff_value = 1;
          if(decoder_enable && bitValue == 0){
            bit_stuff_error = 1;
          }
        }
        break;
      case READ1:
        if (bitValue == 1 && count1 < 5) {
          count1++;
        }
        else if (bitValue == 0 && count1 < 5) {
          statesBitStuff = READ0;
          count0++;
          count1 = 0;
        }
        else if (count1 == 5) {
          statesBitStuff = BIT_STUFF;
          bit_stuff = 1;                  //indica a ocorrencia de um bit stuff
          bit_stuff_value = 0;
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

void bitStuff_E(uint8_t bitValue_E) {
  if (bit_stuff_enable_E == 1) {
   // Serial.println(String(statesBitStuff_E) + " bit_stuff_E " + String(bit_stuff_E) + " count0_E = " + String(count0_E) + " count1_E = " + String(count1_E) );
    switch (statesBitStuff_E) {
      case START:
        count0_E = 0;
        count1_E = 0;
        bit_stuff_E = 0;    //variavel responsavel por indicar a ocorrencia de um bit stuff

        if (bitValue_E == 0) {
          statesBitStuff_E = READ0;
          count0_E++;
        }
        else {
          statesBitStuff_E = READ1;
          count1_E++;
        }
        break;
      case READ0:
        if (bitValue_E == 0 && count0_E < 5) {
          count0_E++;
        }
        else if (bitValue_E == 1 && count0_E < 5) {
          statesBitStuff_E = READ1;
          count1_E = 0;
          count1_E++;
          count0_E = 0;
        }
        else if (count0_E == 5) {
          statesBitStuff_E = BIT_STUFF;
          bit_stuff_E = 1;                  //indica a ocorrencia de um bit stuff
          bit_stuff_value_E = 1;
        }
        break;
      case READ1:
        if (bitValue_E == 1 && count1_E < 5) {
          count1_E++;
        }
        else if (bitValue_E == 0 && count1_E < 5) {
          statesBitStuff_E = READ0;
          count0_E++;
          count1_E = 0;
        }
        else if (count1_E == 5) {
          statesBitStuff_E = BIT_STUFF;
          bit_stuff_E = 1;                  //indica a ocorrencia de um bit stuff
        }
        break;
      case BIT_STUFF:
        bit_stuff_E = 0;
        count0_E = 0;
        count1_E = 0;

        if (bitValue_E == 0) {
          statesBitStuff_E = READ0;
          count0_E++;
          count1_E = 0;
        }
        else {
          statesBitStuff_E = READ1;
          count0_E = 0;
          count1_E++;
        }
        break;
    }
  }
}

//implementacao da Maquina de estados do decoder
void decoderLogic(uint8_t bitValue) {

  switch (statesDecoder) {
    case START_D:
      idle = 0;

      enable_crc = 1;
      crc_error = 0;
      resetCRC();

      if (bitValue == 1) {
        count = 0;  
        bit_stuff = 0;
        er_count = 0;
        bit_stuff_enable = 0;
      }
      else {
        Serial.print("\nStart_D: "); //print
        Serial.println(start_d);  //print
        statesDecoder = ID;
        Serial.print("ID_D: "); //print
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
        Serial.print("\nError Flag: ");
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
      Serial.print("RTR_SRR_D: "); //print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        statesDecoder = IDE;
        rtr_srr = bitValue;
        Serial.println(rtr_srr); //print
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
      }
      break;
    case IDE:
      bitStuff(bitValue);
      Serial.print("IDE_D: "); //print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        ide = bitValue;
        Serial.println(ide); //print
        count = 0;

        if (ide == 1) {
          statesDecoder = ID_EXTEND;
          Serial.print("ID_EXTEND_D: "); //print
        }
        else {
          statesDecoder = R0;
        }
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
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
        Serial.print("\nError Flag: ");
      }
      break;
    case RTR_EXTEND:
      bitStuff(bitValue);
      Serial.print("RTR_EXTEND_D: "); // print
      if (bit_stuff == 0 && bit_stuff_error == 0) {
        rtr_extend = bitValue;
        Serial.println(rtr_extend); //print
        statesDecoder = R1;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
      }
      break;
    case R1:
      bitStuff(bitValue);

      if (bit_stuff == 0 && bit_stuff_error == 0) {
        Serial.print("R1_D: ");
        Serial.println(r1);
        r1 = bitValue;
        statesDecoder = R0;
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
      }
      break;
    case R0:
      bitStuff(bitValue);

      if (bit_stuff == 0 && bit_stuff_error == 0) {
        r0 = bitValue;
        count = 0;
        statesDecoder = DLC;
        Serial.print("R0_D: ");
        Serial.println(r0);
        Serial.print("DLC: ");
      }
      else if (bit_stuff_error == 1) {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
      }
      break;
    case DLC:
      bitStuff(bitValue);
        if(!bit_stuff){
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
            Serial.print("\nError Flag: ");
          }

          if (count == 4 && !bit_stuff) {
            //calcula o datanho do campo de dados
            //        for(int i=0;i<4;i++){
            //          Serial.print("i: ");
            //          Serial.print(i);
            //          Serial.print("dlc: ");
            //          Serial.println(dlc[i]);
            //        }
            data_length = numberOfData(true);
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
        }
      break;
    case DATA:
      bitStuff(bitValue);
      if(!bit_stuff){
        //Serial.println(data_length);
        if (count < data_length && bit_stuff == 0 && bit_stuff_error == 0) {
          data[count] = bitValue;
          Serial.print(data[count]);
        }
        else if (bit_stuff_error == 1 && count < data_length) {     //Verifica se ocorreu um erro enquanto esta lendo os dados
          statesDecoder = ERROR_FLAG;
          Serial.print("\nError Flag: ");
        }

        count ++;
        //Se o tamanho dos dados forem menor que 8 ignora os proximo bit ate mudar de estados
        if (count == data_length) {
          statesDecoder = CRC;
          Serial.println("");
          Serial.print("CRC: ");
          count = 0;
        }
      }
      break;
    case CRC:
      enable_crc = 0;
      bitStuff(bitValue);
      if(!bit_stuff){
        //      Serial.println(count);
        if (count < 14 && bit_stuff == 0 && bit_stuff_error == 0) {
          crc[count] = bitValue;
          Serial.print(crc[count]);
          if(crc_check[count] != crc[count] ) {
            crc_error = 1;
          }
          count++;
        }
        else if (count == 14 && bit_stuff == 0 && bit_stuff_error == 0) {
          crc[count] = bitValue;
          Serial.println(crc[count]);

          statesDecoder = CRC_DELIMITER;
          bit_stuff_enable = 0;         //desabilita a verificacao do bitStuff
          count = 0;
        }
        
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
        Serial.print("\nError Flag: ");
      }
      break;
    case ACK_SLOT:
      ack_slot = bitValue;
       Serial.print("ACK_SLOT: ");
       busWrite(0);
      
      // if(bitValue == 1){
      //    statesDecoder = ERROR_FLAG;
      //    Serial.print("\nError Flag: ");
      // }
      // else{
        statesDecoder = ACK_DELIMITER;
        count = 0;
     // }
          
      

      break;
    case ACK_DELIMITER:
      ack_delimiter = bitValue;
      Serial.print("ACK_DELIMITER: ");
      Serial.println(ack_delimiter);
       if(crc_error == 1){
          // Serial.print("CRC DIFERENTE");
          statesDecoder = ERROR_FLAG;
           Serial.print("\nError Flag: ");
          }     
      else if (ack_delimiter == 1) {
        statesDecoder = ENDOF;
        Serial.print("ENDOF: ");
        count = 0;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
      }
      break;
    case ENDOF:

      if (bitValue == 1 && count < 6) {
        endof[count] = bitValue;
        Serial.print(endof[count]);
        count++;
      }
      else if (bitValue == 1 && count == 6) {
        endof[count] = bitValue;
        Serial.println(endof[count]);
        statesDecoder = IFS;
        Serial.print("IFS: ");
        count = 0;
      }
      else {
        statesDecoder = ERROR_FLAG;
        Serial.print("\nError Flag: ");
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
        Serial.print("\nOverloadFLAG: ");
      }
      idle = 1;
      break;

    case ERROR_FLAG:
      
      if ( er_count < 5) {
        er_count++;
        busWrite(0);
      }
      else if ( er_count == 5) {
                   busWrite(0);
        statesDecoder = ERROR_FLAG2;
        Serial.println("");
        Serial.print("Error Flag2: ");
      }
      break;
    case ERROR_FLAG2:
      if ( bitValue == 1) {
        statesDecoder = ERROR_DELIMITER;
        Serial.println("");
        Serial.print("Erro Delimiter: ");
        count = 1;
      }
      else if (er_count == 11) {
        statesDecoder = ERROR_DELIMITER;
        Serial.println("");
        Serial.print("Erro Delimiter: ");

        count = 0;
      }
      else if ( bitValue == 0 && er_count < 11) {
                   busWrite(0);
        er_count++;
      }
     
      break;
    case ERROR_DELIMITER:
      
      if (count < 7) {
        er_count = 0;
        busWrite(1);
        count++;
      }
      else if (count == 7) {
         
        busWrite(1);
        Serial.println("");
        Serial.print("IFS: ");
        statesDecoder = IFS;
        count = 0;
      }
      break;

    case OVERLOAD_FLAG:
      if (over_count < 5) {
        over_count++; 
        busWrite(0);
      }
      else if (over_count == 5) {
         
         busWrite(0);
        statesDecoder = OVERLOAD_FLAG2;
        Serial.print("\nOverloadFLAG2: ");
      }
      break;
    case OVERLOAD_FLAG2:

      if ( bitValue == 1) {
        statesDecoder = OVERLOAD_DELIMITER;
         Serial.print("\nOverloaddELIMITER: ");
        count = 1;
      }
      else if (bitValue == 0 && over_count < 11) {
         
         busWrite(0);
         over_count++;
      }
      else if (over_count == 11) {
         
         busWrite(0);
        statesDecoder = OVERLOAD_DELIMITER;
        count = 0;
      }
      break;
    case OVERLOAD_DELIMITER:

      if ( count < 7) {
        over_count=0;
        busWrite(1);
        count++;
      }
      else if ( count == 7) {
         
        busWrite(1);

        statesDecoder = IFS;
        Serial.print("\nIFS: ");
        count = 0;
      }
      break;
    }
  if(enable_crc == 1 && !bit_stuff){
    crc_calculator(bitValue);
  }
  if(bit_stuff){
    //Serial.print("!"+ String(bit_stuff_value) + "!");
    bitStuff(bit_stuff_value);
    
  }
}


void encoderLogic(uint8_t bitValue) {
  switch (statesEncoder) {
    case START_D:
      idle = 0;

      Serial.print("\nSTART_E: ");
      enable_crc = 1;
      resetCRC();
      //Serial.println("Entrou");
      arbitration = 1; // habilita arbitração
    
      count_e = 0;  
      bit_stuff_E = 0;

      er_count = 0;
      bit_stuff_enable_E = 0;
      
      busWrite(bitValue);

      statesEncoder = ID;
      Serial.print("\nID_E: ");
      bit_stuff_enable_E = 1;
      bitStuff_E(bitValue);
      start_d = bitValue;
      count_e = 0;
      Serial.print(" ");
      
      break;
    case ID:
      bitStuff_E(bitValue);
      if (bit_stuff_E) {
        busWrite(bit_stuff_value_E); //VERIFICAR COMO RECOMEÇAR DO MESMO BITVALUE
        j--;
      }
      else {
        if (count_e < 10 && bit_stuff_E == 0) {

          busWrite(bitValue);
          id[count_e] = bitValue;
          count_e++;
        }

        else if (count_e == 10 && bit_stuff_E == 0) {
          statesEncoder = RTR_SRR;
          busWrite(bitValue);
          id[count_e] = bitValue;
          Serial.print(" ");
          count_e++;
        }
      }
      
      break;
    case RTR_SRR:
      bitStuff_E(bitValue);
      
      Serial.print("\nRTR_SRR_E: ");
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);     //VERIFICAR COMO RECOMEÇAR DO MESMO BITVALUE
        j--;
      }
      else{
        rtr_srr_e = bitValue;
        statesEncoder = IDE;
        busWrite(bitValue);
      }
      break;
    case IDE:
      bitStuff_E(bitValue);

      Serial.print("\nIDE_E: ");
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else{
        arbitration = 0;
        ide_e = bitValue;
        busWrite(bitValue);
        count_e = 0;

        if (ide_e == 1) {
          statesEncoder = ID_EXTEND;
          Serial.print("\nID_EXTEND_E: ");
        }
        else {
          statesEncoder = R0;
        }
      }
      break;
    case ID_EXTEND:
      bitStuff_E(bitValue);
      arbitration = 1;

      
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else if (count_e < 17) {
        busWrite(bitValue);
        count_e++;
      }
      else if (count_e == 17) {
        busWrite(bitValue);
        statesEncoder = RTR_EXTEND;
        Serial.print(" ");
      }
      break;
    case RTR_EXTEND:
      Serial.println("\nRTR_Extend_E: ");
      arbitration = 0;
      bitStuff_E(bitValue);
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else{
        busWrite(bitValue);
        rtr_extend = bitValue;
        statesEncoder = R1;
      }
      break;
    case R1:
      bitStuff_E(bitValue);
      decoder_enable = 0;
      statesDecoder = START_D;

      Serial.print("\nR1_E: ");
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else{
        r1 = bitValue;
        busWrite(bitValue);
        statesEncoder = R0;
      }
      break;
    case R0:
      Serial.print("\nR0_E: ");
      bitStuff_E(bitValue);
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else{
        busWrite(bitValue);
        count_e = 0;
        statesEncoder = DLC;
        Serial.print("\nDLC: ");
      }
      break;
    case DLC:
      bitStuff_E(bitValue);
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      else if (count_e <= 3) {
        dlc_e[count_e] = bitValue;
        busWrite(bitValue);
        count_e ++;
      }

      if (count_e == 4) {
        //calcula o datanho do campo de dados
        //        for(int i=0;i<4;i++){
        //          Serial.print("i: ");
        //          Serial.print(i);
        //          Serial.print("dlc: ");
        //          Serial.println(dlc[i]);
        //        }
        data_length_e = numberOfData(false);
        // Serial.print("Tamanho dos dados: ");
        // Serial.print(data_length_e);

        if ( ( (rtr_srr_e == 1 && ide_e == 0) || (ide_e == 1 && rtr_extend == 1) ) || data_length_e == 0)  {
          Serial.println("Entrou");
          statesEncoder = CRC;
          Serial.print("CRC: ");
          count_e = 0;
        }
        else {
          statesEncoder = DATA;
          Serial.print("\nDATA: ");
          count_e = 0;
        }
      }
      break;
    case DATA:
      bitStuff_E(bitValue);
      if(bit_stuff_E){
        busWrite(bit_stuff_value_E);
        j--;
      }
      if ( count_e < data_length_e && bit_stuff_E == 0){
        busWrite(bitValue);
        count_e ++;
      }
      //Se o tamanho dos dados forem menor que 8 ignora os proximo bit ate mudar de estados
      if (count_e == data_length_e) {
        statesEncoder = CRC;
        Serial.println("");
        Serial.print("CRC: ");
        count_e = 0;

      }
      break;
    case CRC:
      enable_crc = 0;
      bitStuff_E(crc_check[count_e]);
      if(bit_stuff_E){
        bitStuff_E(bit_stuff_value_E);  //o bit de bit stuff tambem conta para o novo bitstuff
        j--;//se tiver realizado o bitstuff o encoder nao deve avançar no frame
      }
      else if (count_e < 14) {
        busWrite(crc_check[count_e]);
        count_e++;
      }
      else if (count_e == 14) {
        busWrite(crc_check[count_e]);
        statesEncoder = CRC_DELIMITER;
        bit_stuff_enable_E = 0;         //desabilita a verificacao do bitStuff_E
        count_e = 0;
        Serial.print(" ");
      }
      break;
    case CRC_DELIMITER:
      Serial.print("\nCRC_Delimiter");
      busWrite(1);
      statesEncoder = ACK_SLOT;
      break;
    case ACK_SLOT:
      Serial.print("\nAck_Slot: ");
      busWrite(1);
      statesEncoder = ACK_DELIMITER;
      count_e = 0;
      break;
    case ACK_DELIMITER:
      Serial.print("\nAckDelimiter: ");
      busWrite(1);
      statesEncoder = ENDOF;
      Serial.print("\nEOF: ");
      count_e = 0;
      break;
    case ENDOF:
      if ( count_e < 6) {
        busWrite(1);
        count_e++;
      }
      else if ( count_e == 6) {
        busWrite(bitValue);
        statesEncoder = IFS;
        Serial.print("\nIFS: ");
        count_e = 0;
      }
      break;
    case IFS:
      if (count_e < 2) {
        busWrite(1);
        count_e++;
      }
      else if (count_e == 2) {
        //busWrite(ifs[count_e]);
        busWrite(1);
        idle = 1;
        statesEncoder = START_D;
        count_e = 0;
      }
      else if (bitValue == 0) {
        count_e = 0;
        Serial.println("");
        statesEncoder = OVERLOAD_FLAG;
      }
      //Serial.println("");
      break;



    case ERROR_FLAG:

      if ( er_count < 5) {
        er_count++;
        busWrite(0);
      }
      else if ( er_count == 5) {
                   busWrite(0);
        statesDecoder = ERROR_FLAG2;
        Serial.println("");
        Serial.print("Error Flag2: ");
      }
      break;
    case ERROR_FLAG2:
      if ( bitValue == 1) {
        statesDecoder = ERROR_DELIMITER;
        Serial.println("");
        Serial.print("Erro Delimiter: ");
        count_e = 1;
      }
      else if (er_count == 11) {
        statesDecoder = ERROR_DELIMITER;
        Serial.println("");
        Serial.print("Erro Delimiter: ");

        count_e = 0;
      }
      else if ( bitValue == 0 && er_count < 11) {
                   busWrite(0);
        er_count++;
      }
     
      break;
    case ERROR_DELIMITER:
      
      if (count_e < 7) {
         
        busWrite(1);
        count_e++;
      }
      else if (count_e == 7) {
         
        busWrite(1);
        Serial.println("");
        Serial.print("IFS: ");
        i = LENGTHFRAME;
        statesDecoder = IFS;
        count_e = 0;
      }
      break;

    case OVERLOAD_FLAG:
      if (over_count < 5) {
        over_count++; 
        busWrite(0);
      }
      else if (over_count == 5) {
         
         busWrite(0);
        statesDecoder = OVERLOAD_FLAG2;
        Serial.print("\nOverloadFLAG2: ");
      }
      break;
    case OVERLOAD_FLAG2:

      if ( bitValue == 1) {
        statesDecoder = OVERLOAD_DELIMITER;
         Serial.print("\nOverloaddELIMITER: ");
        count_e = 1;
      }
      else if (bitValue == 0 && er_count < 11) {
         
         busWrite(0);
        er_count++;
      }
      else if (er_count == 11) {
         
         busWrite(0);
        statesDecoder = OVERLOAD_DELIMITER;
        count_e = 0;
      }
      break;
    case OVERLOAD_DELIMITER:

      if ( count_e < 7) {
         
        busWrite(1);

        count_e++;
      }
      else if ( count_e == 7) {
         
        busWrite(1);

        statesDecoder = IFS;
        Serial.print("\nIFS: ");
        count_e = 0;
      }
      break;
    }
  if(enable_crc == 1 && !bit_stuff_E){
    crc_calculator(bitValue);
  }
  if(bit_stuff_E){
    //Serial.print("!"+ String(bit_stuff_value_E) + "!");
    bitStuff_E(bit_stuff_value_E); 
  }
  if(arbitration && diff_rx_tx){
    encoder_enable = 0;
    arbitration = 0;
    statesEncoder = START_D;
  }
}


void setup(void){
  Timer1.initialize(TIME_INTR);
  Timer1.attachInterrupt(updateTQ);

  attachInterrupt(digitalPinToInterrupt(RX_PIN), fallingEdgeDetector, FALLING);

  pinMode(HSYNC_PIN, OUTPUT);
  pinMode(STATE0_PIN, OUTPUT);
  pinMode(STATE1_PIN, OUTPUT);
  pinMode(TQ_PIN, OUTPUT);
  pinMode(RX_PIN, INPUT);
  pinMode(IDLE_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  Serial.begin(9600);
  i=0;

}

void loop(void){
  if(idle){
    // decoder_enable = 1;
    encoder_enable = 1;
  }
  btlLogic();
  readAndWriteBus();
  if(encoder_enable && WPFlag){
    encoderLogic(busWriteValue);
    WPFlag = 0;
  }
  if(decoder_enable && sample_pointFlag){
    decoderLogic(busReadValue);
    sample_pointFlag = 0;
  }
  if(encoder_enable && WPFlag){
    encoderLogic(busWriteValue);
    WPFlag = 0;
  }

  
  /*
  uint8_t a[]= {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
  if(i<15) bitStuff(a[i++]);
  bit_stuff_enable = 1;
  



  
  //Teste frame com melhor caso
  uint8_t can_stand_D[] = {0,1,1,0,0,1,1,1,0,0,1,0,0,0,0,1,0,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0,0,0,0,1,0,0,0,0,1,0,1,0,0,0,1,1,0,1,1,1,1,1,1,1,1};////////////////////
  //uint8_t can_stand_D[] = {0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,1,1,0,1,1,1,0,1,0,1,1,0,0,0,0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1};////////////////////
  if (i < sizeof(can_stand_D)/sizeof(can_stand_D[0]) ) {////////////////////
    decoderLogic(can_stand_D[i++]);////////////////////


  } else{
    decoderLogic(1);
  }
*/
  // for(int h = 0; h <15; h++){
  //  Serial.print(crc_check[h]);
  // }
  // Serial.println("");
/*>>>>>>> cfcb45323f0c6a6ec9896ce3f392ad975e30f1b2*/
}