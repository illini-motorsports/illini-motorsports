# illini-motorsports
> Code repository for Illini Motorsports Electronics Subsystem. All code for the microcontrollers and other electronics projects will live here.

![car-image](http://motorsports.illinois.edu/img/car_2014.jpg)

***
## Contributing

### Git Basics
Here is an [interactive Git tutorial](https://try.github.io/levels/1/challenges/1).

You can also skim through [this guide](http://www.vogella.com/tutorials/Git/article.html).

#### Git Basic Workflow
This is an example of what you should do each time you sit down to work on the code. Committing often is preferable to committing infrequently. A good rule of thumb is to commit each time you finish a "feature" or when you finish one particular task in the code you are working on.
- `$ git pull origin master` *(Ensures you have the latest copy of the code on your computer.)*
- Do your work. For example, change `file1.c` and `file2.h`.
- `$ git pull origin master` *(Again, checks for any new commits from other contributors.)*
- `$ git add file1.c file2.h` *(Tells git that you want to commit these, and only these, files.)*
- `$ git commit -m "Enter a good commit message here. (See below)"` *(Commits your changes)*
- `$ git push origin master` *(Pushes your new commit to master branch of the online repository)*

If you are using a GUI, you won't need to use these terminal commands but the process is similiar. It is very important to be sure to only commit the files that you want changed. If a bunch of XML configuration files were automagically changed by MPLAB, for example, you should not commit those changes.

#### Common Problems
[Git Flight Rules](https://github.com/k88hudson/git-flight-rules) Contains answers to many common git questions.

##### Automatic Merge Commits
If you try to commit and something like [this](https://github.com/mass/illini-motorsports/commit/e204ba79dc1cd5ac46f447cf9ac181b125d58260) automatically shows up, you haven't pulled the latest version of the repo. To fix this problem, abort the commit and run this command: `$ git pull --rebase`. This will "rewind" your changes, pull the latest version of the repo, and apply your changes on top of that.

#### Branching
When starting work on a new board's code or a large overhaul of an existing one, it's best to do the work on a separate branch. This way, we can ensure that the `master` branch always has code that compiles and ideally is in a working or semi-working condition. If the change you are going to make is small or can be done in one or two commits, it is fine to do the work on the `master` branch.

##### Example:
```
$ git pull origin master
$ git branch new-branch
$ git checkout new-branch
```
Now you can do all work on and commit to the new branch. When the code is finished, click the "Compare and Pull-Request" button on Github. If the branch can be automagically merged, merge `new-branch` into `master` and delete `new-branch`. If there are merge conflicts, you will have to merge `master` into `new-branch` and fix the conflicts before coming back to the pull request page and finishing the merge.

### Tools and Software

#### [MPLAB X](http://www.microchip.com/pagehandler/en-us/family/mplabx/)
This is the software that is used to program the microcontrollers. You can flash the microcontrollers directly and it also has debugging so you can step through the execution of the program. There are Windows, Linux, and Mac versions.

#### [C18 Compiler](http://www.microchip.com/Developmenttools/ProductDetails.aspx?PartNO=SW006011)
You will also need the C18 compiler, which is specific to the PIC18 family of microcontrollers that we use. It is very outdated but should still work. I beleive the "LITE" version will be enough for our needs, but if not you'll have to get a licenced copy. As far as I know there is only a windows version, so us linux guys get screwed.

#### A Git Client



##### [Git CLI - Linux](http://git-scm.com/download/linux)
If you're an awesome person and on Linux, you can use the command line interface for git. Because you're awesome, I'm assuming you already know how to install things onto your computer.

##### [Git CLI - Windows](http://git-scm.com/download/win)
If you're on Windows and slightly less awesome, but more awesome than average, you can use the Windows command line interface for git.

##### [Git GUI - Mac](https://mac.github.com/)
If you are on Mac and not awesome, you can use Github's Mac app. It should be very easy to use, but I've never actually used it.

##### [Git Plugin for MPLABX](https://code.google.com/p/nbgit/downloads/list?q=0.4)
This will interface directly with MPLAB X. Should also be very easy to use.

##### [Git GUI - Windows](https://windows.github.com/)
If you are on Windows and in the least awesome of all the aforementioned groups, you can use Github's Windows app. Again, it should be easy to use but I've never used it so I can't say one way or the other.

#### [Qt Creator](http://qt-project.org/downloads#qt-creator)
Qt Creator can be used to work on development for the `c++` GUI applications we made (currently: can-translator and telemetry-monitor). It can help with debugging but I don't reccomend using its drag-and-drop mess of a feature to build the actual GUIs, this can be acccomplished much cleaner and easier by just coding it directly. Using Qt Creator isn't necessary, you can also just use [your favorite text editor](http://www.vim.org/) as they are relatively small projects.

### Building

#### [Static Qt Environment](http://qt-project.org/wiki/How-to-build-a-static-Qt-for-Windows-MinGW)
Follow this guide to build a static version of the Qt applications. That will enable them to be run on anyone's windows computer without having to first install Qt on the computer.
 
### Conventions and Style
[This](https://github.com/outsideris/popularconvention) is a good place to start. It doesn't have anything specific to `c++` or `c`, but it can give you a good idea of the right way to go. If you're really bored, you can read [this](http://users.ece.cmu.edu/~eno/coding/CCodingStandard.html) as well.

#### Commit Messages
Commit messages should be useful and descriptive so that someone else on the team knows what was done in the commit. Limit the first line to 72 characters. If you still have more information after that, end your sentence, hit enter twice, and start writing the rest of the message. Generally, commit messages should be written in the imperative. For example `Fix terrible bug.` or `Add awesome feature.`, instead of `Adding new thing` or `I fixed something.`. End your commit messages with a period. If the commit is a work-in-progress, add `WIP.` before your message. I know nobody wants to take the time to do this properly, but it will really make life easier for the whole team in the end.

[Example](https://github.com/flytenow/icarus/commit/6091d42e9136d3b50e20f5582f72c204d1a88bea)

#### `TODO`: Add more conventions. 

***
## Archive
Code for previous competition years is also stored in this repository. It can be found in separate branches. Keep these branches separate and orphaned from `master` (never merge them in).

At the end of a competition year, we will make a new branch off of the `master` branch to archive that year's code. This way, we have a backup of roughly the same code that was run at the last competition for that year.

| Branch | Contents |
| --- | --- |
| [![DAQ-2014](http://img.shields.io/badge/DAQ-2014-orange.svg?style=flat)](https://github.com/mass/illini-motorsports/tree/DAQ-2014) | Code from the 2013-2014 year. |
| [![DAQ-2013](http://img.shields.io/badge/DAQ-2013-orange.svg?style=flat)](https://github.com/mass/illini-motorsports/tree/DAQ-2013) | Code from the 2012-2013 year. |
| [![Wheel-2013](http://img.shields.io/badge/Wheel-2013-orange.svg?style=flat)](https://github.com/mass/illini-motorsports/tree/Wheel-2013) | Code for the steering wheel for the 2012-2013 year. |
| [![PostProcessing-2013](http://img.shields.io/badge/Post_Processing-2013-orange.svg?style=flat)](https://github.com/mass/illini-motorsports/tree/Post_Processing-2013) | Old version of Translate_CAN for the 2012-2013 year. |
| [![DAQ-2012](http://img.shields.io/badge/DAQ-2012-orange.svg?style=flat)](https://github.com/mass/illini-motorsports/tree/DAQ-2012) | Arduino sketches for data aquisition and the steering wheel for the 2011-2012 year. |

***
## License
```
The MIT License (MIT)

Copyright (c) 2014 Illini Motorsports

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
