![GitHub tag (latest SemVer)](https://img.shields.io/github/tag/zerokol/eFLL.svg)
![GitHub](https://img.shields.io/github/license/zerokol/eFLL.svg)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/ad6d2e4416f74e93a104b795d69e26c4)](https://www.codacy.com/app/zerokol/eFLL?utm_source=github.com&utm_medium=referral&utm_content=zerokol/eFLL&utm_campaign=Badge_Grade)
![GitHub top language](https://img.shields.io/github/languages/top/zerokol/eFLL.svg)
![GitHub search hit counter](https://img.shields.io/github/search/zerokol/eFLL/fuzzy.svg)
![GitHub last commit (branch)](https://img.shields.io/github/last-commit/zerokol/eFLL/master.svg)

## eFLL (Embedded Fuzzy Logic Library)

eFLL (Embedded Fuzzy Logic Library) is a standard library for Embedded Systems to implement easy and efficient Fuzzy Systems.

Para informações avançadas, documentação e exemplos de uso em PORTUGUÊS: [eFLL - Uma Biblioteca Fuzzy para Arduino e Sistemas Embarcados](https://blog.zerokol.com/2012/09/arduinofuzzy-uma-biblioteca-fuzzy-para.html)

For advanced information, documentation, and usage examples in ENGLISH: [eFLL - A Fuzzy Library for Arduino and Embedded Systems](https://blog.zerokol.com/2012/09/arduinofuzzy-fuzzy-library-for-arduino.html)

## Characteristics

Written in C++/C, uses only standard C language library "stdlib.h", so eFLL is a library designed not only to Arduino, but any Embedded System or not how have your commands written in C.

It has no explicit limitations on quantity of Fuzzy, Fuzzy Rules, Inputs or Outputs, these limited processing power and storage of each microcontroller

It uses the process:

(MAX-MIN) and (Mamdani Minimum) for inference and composition, (CENTER OF AREA) to defuzzification in a continuous universe.

Tested with [GTest](http://code.google.com/p/googletest/) for C, Google Inc.

## How to install (general use)

**Step 1:** Go to the official project page on GitHub (Here)

**Step 2:** Make a clone of the project using Git or download at Download on the button "Download as zip."

**Step 3:** Clone or unzip (For safety, rename the folder to "eFLL") the files into some folder

**Step 4:** Compile and link it to your code (See Makefile)

## How to install (and import to use with Arduino)

### Easy Way

**Step 1:** Open the Arduino IDE

**Step 2:** In main menu, go to SKETCH >> INCLUDE LIBRARY >> MANAGE LIBRARIES

**Step 3:** Search for "eFLL" or "Fuzzy"

**Step 4:** eFLL will appear in the list, to finish, just click in INSTALL, now you can include eFLL to your sketchs

### Old Way

**Step 1:** Go to the official project page on GitHub (Here)

**Step 2:** Make a clone of the project using Git or download at Download on the button "Download as zip."

**Step 3:** Clone or unzip (For safety, rename the folder to "eFLL") the files into Arduino libraries' folder:

Ubuntu (/usr/share/arduino/libraries/) if installed via apt-get, if not, on Windows, Mac or Linux (where you downloaded the Arduino IDE, the Library folder is inside)

**Ok! The library is ready to be used!**

If the installation of the library has been successfully held, to import the library is easy:

**Step 4:** Open your Arduino IDE, check out the tab on the top menu SKETCH → LIBRARY → Import eFLL

## Brief Documentation

![Class Diagram](https://raw.githubusercontent.com/zerokol/eFLL/master/uml/class-diagram.png)

**Fuzzy object** - This object includes all the Fuzzy System, through it, you can manipulate the Fuzzy Sets, Linguistic Rules, inputs and outputs.

**FuzzyInput** object - This object groups all entries Fuzzy Sets that belongs to the same domain.

**FuzzyOutput** object - This object is similar to FuzzyInput, is used to group all output Fuzzy Sets thar belongs to the same domain.

**FuzzySet** object - This is one of the main objects of Fuzzy Library, with each set is possible to model the system in question. Currently the library supports triangular membership functions, trapezoidal and singleton, which are assembled based on points A, B, C and D, they are passed by parameter in its constructor FuzzySet(float a, float b, float c, float d).

**FuzzyRule** object - This object is used to mount the base rule of Fuzzy object, which contains one or more of this object. Instantiated with FuzzyRule fr = new FuzzyRule (ID, antecedent, consequent).

**FuzzyRuleAntecedent** object - This object is used to compound the object FuzzyRule, responsible for assembling the antecedent of the conditional expression of a FuzzyRule.

**FuzzyRuleConsequent** object - This object is used to render the object FuzzyRule, responsible for assembling the output expression of a FuzzyRule.

## Tips

These are all eFLL library objects that are used in the process. The next step, generally interactive is handled by three methods of the Fuzzy Class first:

`bool setInput(int id, float value);`

It is used to pass the Crispe input value to the system note that the first parameter is the FuzzyInput object' ID which parameter value is intended.

`bool fuzzify();`

It is used to start the fuzzification process, composition and inference.

And finally:

`float defuzzify(int id);`

## REFERENCES

**Authors:** AJ Alves <aj.alves@zerokol.com>; **Co authors:** Dr. Ricardo Lira <ricardor_usp@yahoo.com.br>, Msc. Marvin Lemos <marvinlemos@gmail.com>, Douglas S. Kridi <douglaskridi@gmail.com>, Kannya Leal <kannyal@hotmail.com>

## Special Thanks to Contributors

[@mikebutrimov](https://github.com/mikebutrimov), [@tzikis](https://github.com/tzikis), [@na7an](https://github.com/na7an)

## LICENSE

MIT License
