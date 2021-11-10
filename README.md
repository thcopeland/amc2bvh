## About

`amc2bvh` is a utility that converts a pair of files, one in the Acclaim Skeleton Format (ASF) and the other in the Acclaim Motion Capture (AMC) format, to a single file in the Biovision Hierarchy (BVH) format. This is useful because more modern programs support BVH files than ASF/AMC files.

A previous program called `amc2bvh` used to exist, however, now that its site has gone offline, the only remnant is a Windows executable saved at https://github.com/sxaxmz/amc2bvh. This incarnation attempts to provide more or less the same functionality in an open-source utility.

The most significant source of ASF/AMC files is the [Carnegie Mellon University motion capture database](http://mocap.cs.cmu.edu/). However, Bruce Hahne at [cgspeed](https://sites.google.com/a/cgspeed.com/cgspeed) has already converted them into [BVH files suitable for various animation programs](https://sites.google.com/a/cgspeed.com/cgspeed/motion-capture). You should probably use them in preference of converting them with `amc2bvh`.

If you're interested in learning more about the ASF/AMC or BVH formats, I've found [this website](https://research.cs.wisc.edu/graphics/Courses/cs-838-1999/Jeff/MoCapTOC.html) to be an invaluable reference.

## Installation

#### Windows

Download the most recent version for your operating system from the [Releases](https://github.com/thcopeland/amc2bvh/releases) page. If you're running 64-bit Windows, choose the `x86_64_windows.zip` version. If you're running 32-bit Windows, or you're not sure, choose the `i686_windows.zip` version. After downloading, unzip the folder. Optionally, add the installation directory (something like `%USERPROFILE%\Downloads\amc2bvh-1.0.0_i686_windows`) to your PATH ([instructions](https://www.architectryan.com/2018/03/17/add-to-the-path-on-windows-10/)). To use `amc2bvh`, open the command line and type
```
C:\Users\Tom> Downloads\amc2bvh-1.0.0_i686_windows\amc2bvh.exe FILE.asf FILE.amc -o FILE.bvh
```
If you added the installation directory to your PATH, you can forego the leading `Downloads\amc2bvh-1.0.0_i686_windows\` when you run `amc2bvh`.

Alternatively, you can build from source. `amc2bvh` doesn't use any non-standard libraries.

#### Unix systems

If you're running a x86_64 OS, you can download a binary from the [Releases](https://github.com/thcopeland/amc2bvh/releases) page. If you're running something else or prefer to build from source, clone the repository and run `make`. If you want, copy or symlink it to `$HOME/bin/` or somewhere else on your `$PATH`.

#### Other

Just compile the code or something. Chances are you do this a lot already.

## Usage

`amc2bvh` takes two input files, an AMC and an ASF file, and outputs a single BVH file. It also takes several flags:

```
 $ amc2bvh 06.asf 06_15.amc                     # convert the files
 $ amc2bvh 06.asf 06_15.amc -f 60               # set the playback rate to 60 FPS
 $ amc2bvh 06.asf 06_15.amc -c 8                # allow bones to have up to 8 children
 $ amc2bvh 06.asf 06_15.amc -o basketball.bvh   # place the result in basketball.bvh
 $ amc2bvh 06.asf 06_15.amc --help              # show the help message
```

Although there's room for improvement, `amc2bvh` is plenty fast. It converts a 5594-frame animation on a 30-bone skeleton in about a third of a second, most of which is spend in IO calls.

#### Caveats

- `amc2bvh` performs a straightforward, one-to-one conversion from ASF/AMC files to BVH files. One consequence of this is that the resulting BVH file may contain bones of zero length. I have not found this to be a serious issue, but it causes some importers (Blender, in particular) produce warnings. If this proves to be a problem, we could patch it by setting the bone lengths to some small nonzero value.

- For simplicity, `amc2bvh` allows bones to have at most a fixed number of children, specified by the `-c` flag. The default value is 6, which should be more than sufficient for human models. However, if you get an error like `Error: Bone 'root' has X children, max permitted is Y`, pass `-c X` on the command line.

## License (MIT)

Copyright 2021 Tom Copeland.

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
