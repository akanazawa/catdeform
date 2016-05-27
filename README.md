# Learning 3D Deformation of Animals from 2D Images

A Matlab implementation of the paper ["Learning 3D Deformation of Animals from 2D Image"](http://www.umiacs.umd.edu/~kanazawa/papers/cat_eg2016.pdf)

## Steps
0. Get this repo: `git clone git@github.com:akanazawa/catdeform.git`
1. Download data from
   [this link](http://www.umiacs.umd.edu/~kanazawa/code/catdeform_data.tar.gz),  it contains the 10 images of horses and cats
   along with their annotations. 
2. Download toolbox from [this link](http://www.umiacs.umd.edu/~kanazawa/code/catdeform_toolboxes.tar.gz)
3. Untar them, put `data/` and `toolbox/` in the project root so it is in the same directory
   as this README.md
4. Make sure you have the dependencies as specified below & change the path in
   `code/initialize.m` accordingly
5. Start matlab in `code/`
6. Try:
   - `script_joint_cat`/`script_joint_horse` for solving for the deformation & stiffness over 10 cats/horses
     (as in the paper)
   - `script_cat` : Initial settings is on `Uniform` (no stiffness
     learning) solving for one cat image (best for testing your environment is
     setup properly), can be modified for reusing learned stiffness (see the file)
   - `script_run_many_cat`/`script_run_many_horse` for the ablation study 


## Results in the paper
Can be downloaded from
[this link](http://www.umiacs.umd.edu/~kanazawa/code/catdeform_results.tar.gz). You
need to untar and put `results/` directory in the project root (along with this
README), or compute it yourself for re-use stiffness experiments (that takes in
pre-computed stiffness & solves for deformation).

The `results/` include results for 10 cats and horses with stiffness shown in the
paper. For cats there the results obtained with UNIFORM bound setting  is also
supplied (those under prefix `uniformC`). Those for horse can be computed by running `script_run_many_horse`.


## Dependencies:
Requires MATLAB, tested on 64-bit Linux: Ubuntu 14.04, MATLAB R2015a. 

Install [MOSEK](http://users.ics.forth.gr/~lourakis/sba/) & change the path in `initialize.m` accordingly.
Make sure to use the latest version of YALMIP (included in `toolbox` 2015/09/19 release) otherwise the optimization will be slower.

Windows users: Untested but you can probably use it if pre-compiled binaries of
iso2mesh in `toolboxes/iso2mesh/bin/` and sba matlab (in
`toolboxes/sba_1.5/matlab`) work.

OS X users: Binaries of iso2mesh didn't work for me on El Capitan and couldn't
compile SBA on OS X, but if you really want to make it work please compile them
by following the information on [iso2mesh](http://iso2mesh.sourceforge.net/cgi-bin/index.cgi) and [sba](http://iso2mesh.sourceforge.net/cgi-bin/index.cgi). 


#### Compiling SBA
In case `sba.mexa64` doesn't work fro you, compile it by:

You need to got to
`toolbox/sba_1.5/matlab`

and do 

`make`

It requires BLAS and LAPACK. What worked for me requires these packages:

`sudo apt-get install libblas-dev f2c liblapack-pic`

Change the paths to your libraries in `toolbox/sba_1.5/matlab/Makefile` accordingly.

## Notes:
- code for rendering results is in `code/util/figure_code`
- Use [vtk](www.vtk.org) to visualize the result saved from each iteration, or
  convert it to `.off` using `code/util/vtk2off`

## Bibtex
Please cite our papaer if this code was useful to you.
```
@inproceedings{Kanazawa:2016,
author = {Angjoo Kanazawa and Shahar Kovalsky and Ronen Basri and David W. Jacobs},
title = {Learning 3D Deformation of Animals from 2D Images},
booktitle = {Proceedings of EUROGRAPHICS/ACM SIGGRAPH Symposium on Geometry Processing},
year = {2016},
}
```

## Disclamer:
The code is provided as-is for academic use only and without any guarantees. Please contact the author to report any bugs. Written by
   [Angjoo Kanazawa](http://www.umiacs.umd.edu/~kanazawa/) based on [Controlling Singular Values code](https://github.com/shaharkov/ContSingVal) by [Shahar Kovalsky](http://www.wisdom.weizmann.ac.il/~shaharko/) and [Noam Aigerman](http://www.wisdom.weizmann.ac.il/~noamaig/real_home.html)
