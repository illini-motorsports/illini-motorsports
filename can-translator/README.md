# can-translator [![Build Status](https://travis-ci.org/mass/can-translator.png?branch=master)](https://travis-ci.org/mass/can-translator) ![GA Beacon](https://ga-beacon.appspot.com/UA-43696434-2/can-translator/readme)

> Post processing program to convert raw data from the CAN bus into a format that can be imported into WinDarab for analysis.

## Changelog
| Release | Changes |
| --- | --- |
| `0.1.0` | Configuration Scanning Complete |
| `0.0.1` | Repository Creation |

## Dependencies
This project requires several libraries to build properly on Linux. If you are using Windows, good luck.
- build-essential
- libqt4-dev

## Building
First, navigate to the top level directory containing the source code.

Run `qmake` and then run `make`.

This should generate an executable file called `can-translator` which, when executed, will start the program and open up the GUI.

## Travis-CI
This project is configured to build after each commit through the use of Travis-CI. The builds are tested with gcc and clang.

## Bugs and Issues
For bugs, questions, and discussions please use [Github Issues](https://github.com/mass/can-translator/issues).

## License
```
The MIT License (MIT)

Copyright (c) 2014 Andrew Mass

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER  
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
