# illini-motorsports
> Code repository for Illini Motorsports Electronics Subsystem. All code for the microcontrollers and other electronics projects will live here.

![car-image](http://motorsports.illinois.edu/img/car_2014.jpg)

## Contributing

### Branching
When starting work on a new board's code or a large overhaul of an existing one, it's best to do the work on a separate branch. This way, we can ensure that the `master` branch always has code that compiles and ideally is in a working or semi-working condition. If the change you are going to make is small or can be done in one or two commits, it is fine to do the work on the `master` branch.

#### Example workflow:
```
$ git pull origin master
$ git branch new-branch
$ git checkout new-branch
```
Now you can do all work on and commit to the new branch. When the code is finished, click the "Compare and Pull-Request" button on Github. If the branch can be automagically merged, merge `new-branch` into `master` and delete `new-branch`. If there are merge conflicts, you will have to merge `master` into `new-branch` and fix the conflicts before coming back to the pull request page and finishing the merge.

### Tools and Software
`todo`
 
### Conventions and Style
`todo`

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



