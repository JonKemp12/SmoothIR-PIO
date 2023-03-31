
#include "Arduino.h"

class MyClass {
  private:
    static int counter[2];
    int instanceIndex;
  public:
    MyClass(int index) {
      instanceIndex = index;
    }
    static void interruptHandler1() {
      counter[0]++;
    }
    static void interruptHandler2() {
      counter[1]++;
    }
    int getCounter() {
      return counter[instanceIndex];
    }
};

// Initialize static data members
int MyClass::counter[2] = {0};

// Attach interrupts to the handlers
attachInterrupt(digitalPinToInterrupt(2), MyClass::interruptHandler1, CHANGE);
attachInterrupt(digitalPinToInterrupt(3), MyClass::interruptHandler2, CHANGE);

// Create two instances of MyClass
MyClass myClassInstance1(0);
MyClass myClassInstance2(1);

// Get the counter values
int counterValue1 = myClassInstance1.getCounter();
int counterValue2 = myClassInstance2.getCounter();
