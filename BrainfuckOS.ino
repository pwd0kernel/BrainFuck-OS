/********************************************************************
 * BrainfuckOS.ino
 *
 * A Brainfuck "OS" for Arduino Uno:
 *  - Multi-tasking BF interpreter (up to 2 tasks)
 *  - EEPROM-based "filesystem" (4 slots)
 *  - Shell commands for code loading, saving, pin control, etc.
 *  - Memory optimizations: struct packing, smaller buffers, PROGMEM strings
 *
 * (c) 2024 pwd0kernel, Licensed under MIT
 ********************************************************************/

#include <Arduino.h>
#include <EEPROM.h>

// ------------------ CONFIGURATIONS ------------------
#define SERIAL_BAUD       9600

// Tasks / BF environment
#define MAX_TASKS         2
#define BF_DATA_SIZE      64
#define BFS_MAX_INSTR     64      // max compiled instructions
#define BF_SOURCE_MAX     128     // max raw BF code length

#define BF_LED_PIN        13

// EEPROM-based "FS"
#define EEPROM_SLOT_COUNT 4
#define EEPROM_NAME_SIZE  12      // file name max length
#define EEPROM_SLOT_SIZE  BF_SOURCE_MAX

// We'll store each slot as:
//   [0..(EEPROM_NAME_SIZE-1)]: name (0-terminated if shorter)
//   [EEPROM_NAME_SIZE..(EEPROM_NAME_SIZE+EEPROM_SLOT_SIZE-1)]: BF code

// ---------- Data Types for BFS -----------
enum __attribute__((packed)) BFS_Op : uint8_t {
  BFS_OP_NOP       = 0,
  BFS_OP_MOVE      = 1,
  BFS_OP_PLUS      = 2,
  BFS_OP_OUT       = 3,
  BFS_OP_IN        = 4,
  BFS_OP_LED       = 5,
  BFS_OP_LOOPSTART = 6,
  BFS_OP_LOOPEND   = 7,
  // Extended
  BFS_OP_DWRITE    = 8, // '#'
  BFS_OP_DREAD     = 9  // '~'
};

// Force byte alignment to 1
#pragma pack(push,1)
struct BFS_Instruction {
  int8_t   jumpTo; // -1 => no jump, else index 0..63
  int8_t   count;  // merges e.g. +/- or pointer moves
  BFS_Op   op;     // BFS operation code
};
#pragma pack(pop)

// Task state
enum TaskState : uint8_t {
  TASK_UNUSED  = 0,
  TASK_READY   = 1,
  TASK_RUNNING = 2,
  TASK_STOPPED = 3
};

struct BFTask {
  bool            used;
  TaskState       state;
  BFS_Instruction instr[BFS_MAX_INSTR];
  int8_t          instrCount; 
  uint8_t         data[BF_DATA_SIZE];
  int8_t          dataPtr;   
  int8_t          pc;        
};

// Global tasks
static BFTask  tasks[MAX_TASKS];
static uint8_t currentTaskIndex = 0;

// Shell input buffer
static char    shellLine[32]; // shortened to save memory
static uint8_t shellPos = 0;

// We'll store "last typed BF code" in this global buffer
static char lastBFSource[BF_SOURCE_MAX];
static bool lastBFUsed = false; // indicates if we have something to SAVE

// --------------- PROGMEM Strings ---------------
const char helpText[] PROGMEM =
  "Commands:\n"
  "  NEW <bf>       Create BF task\n"
  "  PS             List tasks\n"
  "  KILL <id>      Kill task\n"
  "  LS             List EEPROM slots\n"
  "  SAVE <s> <nm>  Save last BF code\n"
  "  LOAD <s>       Load BF code from slot\n"
  "  RUN <s>        Same as LOAD\n"
  "  PINMODE <p> <IN/OUT>\n"
  "  DWRITE <p> <0/1>\n"
  "  DREAD <p>\n"
  "  MEM            Show memory info\n"
  "  HELP           Print help\n";

// ----------- Forward Declarations -----------
int8_t  compileBF(const char* bfSource, BFS_Instruction* outInstr, int8_t maxInstr);
void    parseShellCommand(const char* line);
void    runSomeInstructions(uint8_t tID, uint8_t steps);

// --------------- EEPROM Helpers ---------------
static int eepromSlotBase(int slot) {
  return slot * (EEPROM_NAME_SIZE + EEPROM_SLOT_SIZE);
}

static void eepromReadSlotName(int slot, char* nameBuf, int nameBufSize) {
  int base = eepromSlotBase(slot);
  for(int i=0; i<(nameBufSize-1); i++){
    nameBuf[i] = (char)EEPROM.read(base + i);
    if(nameBuf[i]==0) break;
  }
  nameBuf[nameBufSize-1] = 0;
}

static void eepromWriteSlotName(int slot, const char* name) {
  int base = eepromSlotBase(slot);
  int i=0;
  for(; i<EEPROM_NAME_SIZE-1; i++) {
    if(name[i] == 0) {
      EEPROM.update(base + i, 0);
      i++;
      break;
    }
    EEPROM.update(base + i, name[i]);
  }
  for(; i<EEPROM_NAME_SIZE; i++) {
    EEPROM.update(base + i, 0);
  }
}

static void eepromReadSlotBF(int slot, char* bfBuf, int bfBufSize) {
  int base = eepromSlotBase(slot) + EEPROM_NAME_SIZE;
  for(int i=0; i<(bfBufSize-1); i++){
    char c = (char)EEPROM.read(base + i);
    bfBuf[i] = c;
    if(c==0) break;
  }
  bfBuf[bfBufSize-1] = 0;
}

static void eepromWriteSlotBF(int slot, const char* bfCode) {
  int base = eepromSlotBase(slot) + EEPROM_NAME_SIZE;
  int len = strlen(bfCode);
  if(len > (BF_SOURCE_MAX-1)) len = BF_SOURCE_MAX-1;
  int i=0;
  for(; i<len; i++){
    EEPROM.update(base + i, bfCode[i]);
  }
  EEPROM.update(base + i, 0);
  i++;
  for(; i<EEPROM_SLOT_SIZE; i++){
    EEPROM.update(base + i, 0);
  }
}

// ------------- Task Creation -------------
int8_t createTask(const char* bfSrc) {
  // find free slot
  for(uint8_t i=0; i<MAX_TASKS; i++) {
    if(!tasks[i].used) {
      tasks[i].used      = true;
      tasks[i].state     = TASK_READY;
      memset(tasks[i].data, 0, BF_DATA_SIZE);
      tasks[i].dataPtr   = 0;
      tasks[i].pc        = 0;

      int8_t c = compileBF(bfSrc, tasks[i].instr, BFS_MAX_INSTR);
      tasks[i].instrCount = c;

      Serial.print(F("Task #"));
      Serial.print(i);
      Serial.print(F(" with "));
      Serial.print(c);
      Serial.println(F(" BFS instr created."));
      return i;
    }
  }
  Serial.println(F("No free task slots!"));
  return -1;
}

// ------------- BF Compilation -------------
int8_t compileBF(const char* bfSource, BFS_Instruction* outInstr, int8_t maxInstr) {
  int sourceLen = strlen(bfSource);
  int idxOut = 0;
  int i = 0;

  while((i < sourceLen) && (idxOut < maxInstr)) {
    char c = bfSource[i];
    if(c=='>' || c=='<') {
      bool isRight = (c=='>');
      int8_t count=0;
      while(i<sourceLen && bfSource[i] == (isRight?'>':'<')) { i++; count++; }
      if(!isRight) count = -count;
      outInstr[idxOut].op     = BFS_OP_MOVE;
      outInstr[idxOut].count  = count;
      outInstr[idxOut].jumpTo = -1;
      idxOut++;
    }
    else if(c=='+' || c=='-') {
      bool isPlus = (c=='+');
      int8_t count=0;
      while(i<sourceLen && bfSource[i] == (isPlus?'+':'-')) { i++; count++; }
      if(!isPlus) count = -count;
      outInstr[idxOut].op     = BFS_OP_PLUS;
      outInstr[idxOut].count  = count;
      outInstr[idxOut].jumpTo = -1;
      idxOut++;
    }
    else if(c=='.') {
      outInstr[idxOut].op     = BFS_OP_OUT;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c==',') {
      outInstr[idxOut].op     = BFS_OP_IN;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c=='^') {
      outInstr[idxOut].op     = BFS_OP_LED;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c=='[') {
      outInstr[idxOut].op     = BFS_OP_LOOPSTART;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c==']') {
      outInstr[idxOut].op     = BFS_OP_LOOPEND;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c=='#') {
      outInstr[idxOut].op     = BFS_OP_DWRITE;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else if(c=='~') {
      outInstr[idxOut].op     = BFS_OP_DREAD;
      outInstr[idxOut].count  = 1;
      outInstr[idxOut].jumpTo = -1;
      idxOut++; i++;
    }
    else {
      // ignore
      i++;
    }
  }

  // bracket matching
  int8_t stack[16];
  int8_t sp = 0;
  for(int8_t k=0; k<idxOut; k++) {
    if(outInstr[k].op == BFS_OP_LOOPSTART) {
      if(sp>=16) {
        Serial.println(F("Bracket stack overflow!"));
        break;
      }
      stack[sp++] = k;
    }
    else if(outInstr[k].op == BFS_OP_LOOPEND) {
      if(sp>0) {
        int8_t openK = stack[--sp];
        outInstr[openK].jumpTo = k;
        outInstr[k].jumpTo     = openK;
      } else {
        Serial.println(F("Unmatched ']'!"));
      }
    }
  }
  if(sp!=0) {
    Serial.println(F("Unmatched '['!"));
  }

  return idxOut;
}

// ------------- BFS Execution -------------
static void interpretInstruction(BFTask &task, BFS_Instruction &inst) {
  switch(inst.op) {
    case BFS_OP_MOVE: {
      task.dataPtr += inst.count;
      // wrap
      while(task.dataPtr<0)              task.dataPtr += BF_DATA_SIZE;
      while(task.dataPtr>=BF_DATA_SIZE)  task.dataPtr -= BF_DATA_SIZE;
      task.pc++;
    } break;

    case BFS_OP_PLUS: {
      int val = task.data[task.dataPtr] + inst.count;
      val &= 0xFF;
      task.data[task.dataPtr] = (uint8_t)val;
      task.pc++;
    } break;

    case BFS_OP_OUT: {
      Serial.write((char)task.data[task.dataPtr]);
      task.pc++;
    } break;

    case BFS_OP_IN: {
      if(Serial.available()) {
        task.data[task.dataPtr] = (uint8_t)Serial.read();
      } else {
        task.data[task.dataPtr] = 0;
      }
      task.pc++;
    } break;

    case BFS_OP_LED: {
      digitalWrite(BF_LED_PIN, !digitalRead(BF_LED_PIN));
      task.pc++;
    } break;

    case BFS_OP_LOOPSTART: {
      if(task.data[task.dataPtr]==0) {
        task.pc = inst.jumpTo + 1;
      } else {
        task.pc++;
      }
    } break;

    case BFS_OP_LOOPEND: {
      if(task.data[task.dataPtr]!=0) {
        task.pc = inst.jumpTo + 1;
      } else {
        task.pc++;
      }
    } break;

    case BFS_OP_DWRITE: {
      // dataPtr => pin, next => value
      uint8_t pin   = task.data[task.dataPtr];
      int8_t  nxtPtr= task.dataPtr + 1;
      if(nxtPtr >= BF_DATA_SIZE) nxtPtr -= BF_DATA_SIZE;
      uint8_t val   = (task.data[nxtPtr]==0)? LOW : HIGH;
      pinMode(pin, OUTPUT);
      digitalWrite(pin, val);
      task.pc++;
    } break;

    case BFS_OP_DREAD: {
      // dataPtr => pin
      uint8_t pin = task.data[task.dataPtr];
      pinMode(pin, INPUT);
      uint8_t val = digitalRead(pin);
      task.data[task.dataPtr] = val;
      task.pc++;
    } break;

    default: {
      // BFS_OP_NOP or unknown
      task.pc++;
    } break;
  }
}

void runSomeInstructions(uint8_t tID, uint8_t steps) {
  BFTask &T = tasks[tID];
  T.state = TASK_RUNNING;

  while(steps--) {
    if(T.pc<0 || T.pc>=T.instrCount) {
      T.state = TASK_STOPPED;
      break;
    }
    interpretInstruction(T, T.instr[T.pc]);
    if(T.state==TASK_STOPPED) break;
  }
  if(T.state==TASK_RUNNING) {
    T.state = TASK_READY;
  }
}

// ------------- Shell Handling -------------
void handleShell() {
  while(Serial.available()) {
    char c = (char)Serial.read();
    if(c=='\n' || c=='\r') {
      shellLine[shellPos] = '\0';
      shellPos=0;
      parseShellCommand(shellLine);
      Serial.print(F("> "));
    } else {
      if(shellPos < (sizeof(shellLine)-1)) {
        shellLine[shellPos++] = c;
      }
    }
  }
}

void parseShellCommand(const char* line) {
  // quick function to skip whitespace
  auto skipSpaces = [&](const char* s){ 
    while(*s==' '||*s=='\t') s++;
    return s;
  };

  // 1) NEW <bf>
  if(!strncasecmp(line,"NEW ",4)) {
    const char* bfCode = skipSpaces(line+4);
    strncpy(lastBFSource, bfCode, BF_SOURCE_MAX-1);
    lastBFSource[BF_SOURCE_MAX-1] = '\0';
    lastBFUsed = true;
    createTask(bfCode);
  }
  // 2) PS
  else if(!strcasecmp(line,"PS")) {
    for(uint8_t i=0; i<MAX_TASKS; i++){
      if(tasks[i].used) {
        Serial.print(F("Task "));
        Serial.print(i);
        Serial.print(F(": "));
        switch(tasks[i].state){
          case TASK_READY:   Serial.print(F("READY"));   break;
          case TASK_RUNNING: Serial.print(F("RUNNING")); break;
          case TASK_STOPPED: Serial.print(F("STOPPED")); break;
          default:           Serial.print(F("?"));       break;
        }
        Serial.print(F(" pc="));
        Serial.print(tasks[i].pc);
        Serial.print('/');
        Serial.print(tasks[i].instrCount);
        Serial.println();
      }
    }
  }
  // 3) KILL <id>
  else if(!strncasecmp(line,"KILL ",5)) {
    int id = atoi(line+5);
    if(id>=0 && id<MAX_TASKS && tasks[id].used) {
      tasks[id].state = TASK_STOPPED;
      Serial.print(F("Killed task "));
      Serial.println(id);
    } else {
      Serial.println(F("Invalid task ID."));
    }
  }
  // 4) LS
  else if(!strcasecmp(line,"LS")) {
    for(int s=0; s<EEPROM_SLOT_COUNT; s++){
      char nameBuf[EEPROM_NAME_SIZE];
      eepromReadSlotName(s, nameBuf, sizeof(nameBuf));
      Serial.print(F("Slot "));
      Serial.print(s);
      Serial.print(F(" '"));
      Serial.print(nameBuf);
      Serial.print(F("' "));
      char bfBuf[BF_SOURCE_MAX];
      eepromReadSlotBF(s, bfBuf, sizeof(bfBuf));
      int len = strlen(bfBuf);
      Serial.print(F("len="));
      Serial.println(len);
    }
  }
  // 5) SAVE <slot> <name>
  else if(!strncasecmp(line,"SAVE ",5)) {
    const char* p = skipSpaces(line+5);
    int slot = atoi(p);
    if(slot<0 || slot>=EEPROM_SLOT_COUNT) {
      Serial.println(F("Invalid slot."));
      return;
    }
    while(*p && *p!=' ' && *p!='\t') p++;
    p = skipSpaces(p);
    if(!lastBFUsed){
      Serial.println(F("No BF code to save! Use NEW first."));
      return;
    }
    eepromWriteSlotName(slot, p);
    eepromWriteSlotBF(slot, lastBFSource);
    Serial.print(F("Saved code to slot "));
    Serial.println(slot);
  }
  // 6) LOAD <slot>
  else if(!strncasecmp(line,"LOAD ",5)) {
    int slot = atoi(line+5);
    if(slot<0 || slot>=EEPROM_SLOT_COUNT) {
      Serial.println(F("Invalid slot."));
      return;
    }
    char bfBuf[BF_SOURCE_MAX];
    eepromReadSlotBF(slot, bfBuf, sizeof(bfBuf));
    if(!bfBuf[0]){
      Serial.println(F("Empty slot."));
      return;
    }
    strncpy(lastBFSource, bfBuf, BF_SOURCE_MAX-1);
    lastBFSource[BF_SOURCE_MAX-1] = '\0';
    lastBFUsed = true;
    createTask(bfBuf);
  }
  // 7) RUN <slot>
  else if(!strncasecmp(line,"RUN ",4)) {
    int slot = atoi(line+4);
    if(slot<0 || slot>=EEPROM_SLOT_COUNT) {
      Serial.println(F("Invalid slot."));
      return;
    }
    char bfBuf[BF_SOURCE_MAX];
    eepromReadSlotBF(slot, bfBuf, sizeof(bfBuf));
    if(!bfBuf[0]){
      Serial.println(F("Empty slot."));
      return;
    }
    strncpy(lastBFSource, bfBuf, BF_SOURCE_MAX-1);
    lastBFSource[BF_SOURCE_MAX-1] = '\0';
    lastBFUsed = true;
    createTask(bfBuf);
  }
  // 8) PINMODE <pin> <IN/OUT>
  else if(!strncasecmp(line,"PINMODE ",8)) {
    const char* p = skipSpaces(line+8);
    int pin = atoi(p);
    while(*p && *p!=' ' && *p!='\t') p++;
    p = skipSpaces(p);
    if(!strncasecmp(p,"IN",2)) {
      pinMode(pin,INPUT);
      Serial.print(F("Pin "));
      Serial.print(pin);
      Serial.println(F(" -> INPUT"));
    } else if(!strncasecmp(p,"OUT",3)) {
      pinMode(pin,OUTPUT);
      Serial.print(F("Pin "));
      Serial.print(pin);
      Serial.println(F(" -> OUTPUT"));
    } else {
      Serial.println(F("Usage: PINMODE <p> <IN/OUT>"));
    }
  }
  // 9) DWRITE <pin> <0/1>
  else if(!strncasecmp(line,"DWRITE ",7)) {
    const char* p = skipSpaces(line+7);
    int pin = atoi(p);
    while(*p && *p!=' ' && *p!='\t') p++;
    p = skipSpaces(p);
    int val = atoi(p);
    digitalWrite(pin, val==0 ? LOW : HIGH);
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.print(F(" -> "));
    Serial.println(val==0 ? F("LOW") : F("HIGH"));
  }
  // 10) DREAD <pin>
  else if(!strncasecmp(line,"DREAD ",6)) {
    const char* p = skipSpaces(line+6);
    int pin = atoi(p);
    int val = digitalRead(pin);
    Serial.print(F("Pin "));
    Serial.print(pin);
    Serial.print(F(" = "));
    Serial.println(val);
  }
  // 11) MEM
  else if(!strcasecmp(line,"MEM")) {
    Serial.println(F("MEM Info:"));
    for(uint8_t i=0; i<MAX_TASKS; i++){
      if(tasks[i].used){
        Serial.print(F("Task #"));
        Serial.print(i);
        Serial.print(F(" PC="));
        Serial.print(tasks[i].pc);
        Serial.print(F(" DP="));
        Serial.print(tasks[i].dataPtr);
        Serial.println();
      }
    }
  }
  // 12) HELP
  else if(!strcasecmp(line,"HELP")) {
    // Print help text from PROGMEM
    const char* ptr = helpText;
    while(true) {
      char c = pgm_read_byte(ptr++);
      if(c==0) break;
      Serial.write(c);
    }
    Serial.println();
  }
  else {
    Serial.println(F("Unknown cmd. Type HELP."));
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD);
  pinMode(BF_LED_PIN, OUTPUT);
  digitalWrite(BF_LED_PIN, LOW);

  for(uint8_t i=0; i<MAX_TASKS; i++){
    tasks[i].used = false;
  }
  memset(lastBFSource, 0, BF_SOURCE_MAX);
  lastBFUsed = false;

  Serial.println(F("FullShellBFOS Optimized - Type HELP"));
  Serial.print(F("> "));
}

void loop(){
  handleShell();

  // round-robin scheduling
  for(uint8_t i=0; i<MAX_TASKS; i++){
    currentTaskIndex = (currentTaskIndex+1) % MAX_TASKS;
    BFTask &T = tasks[currentTaskIndex];
    if(T.used && T.state==TASK_READY){
      runSomeInstructions(currentTaskIndex,5);
    }
  }
}
