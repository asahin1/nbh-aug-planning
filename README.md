# Neighborhood Augmented Planning

### Last Updated: 07/13/2023

---

## Description:

This repository contains the implementation of the Neighborhood-Augmented Planning algorithm for 2D and 3D environments.

For more details regarding the algorithm:

- [Topo-Geometrically Distinct Path Computation using Neighborhood-augmented Graph, and its Application to Path Planning for a Tethered Robot in 3D](https://arxiv.org/abs/2306.01203) (under review)

## Included Modules

### Discrete Optimal Search Library (DOSL)

Modified versions of some DOSL planners (A*, S* and Theta\*) and utility tools are included in this repository. Original version of the library is available at: [subh83/DOSL](https://github.com/subh83/DOSL)

### Simple OpenGL (SGL)

SGL is included in this repository for 3D visualizations. Some modifications are done to allow adjustment of initial camera view. Original version of the library is available at: [subh83/sgl](https://github.com/subh83/sgl)

SGL contains the GLT-ZPR module. GLT-ZPR:
Zoom-pan-rotate mouse manipulation module for GLUT.
Version 0.4, October 2003
by Nigel Stewart.
License: GNU Lesser General Public License.
Link: http://www.nigels.com/glt/gltzpr/

## Other Dependencies

For 2D: OpenCV

For 3D: See SGL dependencies

## Compilation

1. Clone the repository:

   ```
   git clone https://github.com/asahin1/nbh-aug-planning
   ```

2. Compile 2D or 3D program (2D_NAG_Search or 3D_NAG_Search):

   ```
   cd nbh-aug-planning
   make <program-name>
   ```

3. Default search algorithm is set as SStar, you may also choose AStar by entering the algorithm name when prompted during compilation.

## Running the Program

1. Refer to [expt/2d_environments.json](https://github.com/asahin1/nbh-aug-planning/blob/main/expt/2d_environments.json) and [expt/3d_environments.json](https://github.com/asahin1/nbh-aug-planning/blob/main/expt/3d_environments.json) for descriptions of 2D and 3D environments. You may choose one of the available environments, make any changes to the existing environment descriptions, or add your own.

2. Refer to [expt/algorithm_parameters.json](https://github.com/asahin1/nbh-aug-planning/blob/main/expt/algorithm_parameters.json) for different parameter sets. You may choose one of the available parameter sets, make any changes to existing parameter sets, or add your own.

3. Run the compiled method on a chosen environment with a parameter set:
   ```
   ./bin/<program-name>_<algorithm-name> <env-name> <parameter-set-name>
   ```

## Some Example Runs

We provide following example runs that reproduce some of the figures in our paper:

### 2D Examples

```
cd nbh-aug-planning
make 2D_NAG_Search
```

We use the default algorithm SStar, when prompted.

#### Obstacles only

```
./bin/2D_NAG_Search_SStar map_indoor 2d_original
```

![](/media/map_indoor.png)

#### Obstacles and high cost regions

```
./bin/2D_NAG_Search_SStar map_nonuniform_with_obstacles 2d_original
```

![](/media/map_nonuniform_with_obstacles.png)

#### Extremely low curvature environment (low cost multiplier)

```
./bin/2D_NAG_Search_SStar map_single_high_cost 2d_with_cut
```

![](/media/map_single_high_cost.png)

### 3D Examples

```
cd nbh-aug-planning
make 3D_NAG_Search
```

We use the default algorithm SStar, when prompted.

#### Long rectangular prism

```
./bin/3D_NAG_Search_SStar env_long_rectangle 3d_original
```

![](/media/env_long_rectangle.png)

#### Building-like environment

```
./bin/3D_NAG_Search_SStar env_building2 3d_original
```

![](/media/env_building2.png)

#### Trefoil knot

```
./bin/3D_NAG_Search_SStar env_trefoil_knot 3d_original
```

![](/media/env_trefoil_knot.png)

#### Extremely low curvature environment (around a 3D corner)

```
./bin/3D_NAG_Search_SStar env_cube 3d_with_cut
```

![](/media/env_cube.png)

## Citation

If you found the Neighborhood Augmented Planning algorithm useful in your research, please cite it in your paper as follows:

_Alp Sahin and Subhrajit Bhattacharya, "Topo-Geometrically Distinct Path Computation using Neighborhood-augmented Graph, and its Application to Path Planning for a Tethered Robot in 3D", 2023, arXiv:2306.01203 [cs.RO]._

Bibtex entry:

```
 @misc{sahin2023topogeometrically,
      title={Topo-Geometrically Distinct Path Computation using Neighborhood-augmented Graph, and its Application to Path Planning for a Tethered Robot in 3D},
      author={Alp Sahin and Subhrajit Bhattacharya},
      year={2023},
      eprint={2306.01203},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
