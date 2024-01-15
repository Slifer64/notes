# Calculate inertial of mesh

https://classic.gazebosim.org/tutorials?tut=inertia&cat=build_robot

Open the mesh file in MeshLab. Display the Layers dialog: `View -> Show Layer Dialog`. A panel opens in the right part of the window, where the lower part contains text output.

Compute the inertial parameters: from the menu `Filters -> Quality Measure and Computations->Compute Geometric Measures`. The lower part of the Layers dialog should show some info about the inertial measures.

The mass is taken equal to the volume (implicitly using a density of 1).
The inertia should be multiplied by the measured mass and divided by the computed volume from the text output.

MeshLab only computes correct inertia parameters for closed shapes. If your link is open or if it is a very complex or concave shape, it might be a good idea to simplify the model (e.g. in Blender) before computing the inertial parameters. Or, if you have the collision shapes for your model, use them in place of the full-resolution model.

## Center of Mass and Inertia units

MeshLab may not use the same length units as you'd want (e.g. meters). You can compute ratio of the bounding box size in your desired units over the MeshLab's one. Then, multiply the Center of Mass with the computed ratio.

Similarly, the moment of inertia should be scaled as well, though the scale factor should be squared to account for the length<sup>2</sup> dependence in the inertia.

## Check in `rviz`

Visualize:
- center of mass: `RobotModel -> Mass Properties -> Mass`
- inertia matrix: `RobotModel -> Mass Properties -> Inertia`

The inertia matrix of a link is displayed as a red box with dimensions such that its inertia is equal to the link's inertia matrix (solving for `h,d,w` in <code>m(h<sup>2</sup> + d<sup>2</sup>)/12 = I<sub>xx</sub></code>, <code>m(w<sup>2</sup> + h<sup>2</sup>)/12 = I<sub>yy</sub></code>, <code>m(w<sup>2</sup> + d<sup>2</sup>)/12 = I<sub>zz</sub></code>).
The red box should have more or less the same shape as the bounding box of the link.

## Rotated inertia
To fix a wrongly rotated Inertia Matrix, swap accordingly the <code>I<sub>xx</sub>, I<sub>yy</sub>, I<sub>zz</sub></code> entries. Also, swap appropriately the <code>I<sub>xy</sub></code>, <code>I<sub>xz</sub></code> and <code>I<sub>yz</sub></code> values (when you swap <code>I<sub>xx</sub></code><-><code>I<sub>yy</sub></code>, then you should negate <code>I<sub>xy</sub></code> and swap <code>I<sub>xz</sub></code><-><code>I<sub>yz</sub></code>).

Notice that in a urdf, in the `<inertial>` block, you can specify in `<origin>` the `xyz` fot the center of mass (i.e. the offset w.r.t. to the link's origin). However the `rpy` is ignored! (the inertia orientation is assumed to match that of the link's origin). 

## Duplicate faces
Duplicate faces within a mesh will affect the calculation of volume and moment of inertia. To fix, use: \
`Filters -> Cleaning and Repairing -> Remove Duplicate Faces`

## Increase numerical precision
Meshlab prints with 6 digits of fixed point precision.

To overcome lack of precision, you can scale up the model using:\
`Filters->Normals, Curvatures and Orientation->Transform: Scale`

If you choose a scaling factor `s`, to undo the scaling, multiply the inertia tensor by <code>1/s<sup>5</sup></code>.
(MeshLab uses the volume as a proxy for mass, which will vary as <code>s<sup>3</sup></code>. Furthermore, the inertia has an addition dependence on length<sup>2</sup>, so the moment of inertia, which is m will change according to <code>s<sup>5</sup></code>.)

Also, you must scale the calculated volume `V` to <code>V' = V/s<sup>3</sup></code>, and divide all entries in the inertia matrix by `V'` (`V' = m` as density=1 is assumed) and then in the urdf, multiply them with the actual mass of the link.

The center of mass must also be scaled by <code>1/s</code>.

## Non 'watertight' objects
The inertia cannot be calculated for no 'watertight' objects. To fix this, you can try the following:
- `Filters -> Cleaning and Repairing -> Repair non Manifold Edges`
- `Filters -> Remeshing, Simplification and Reconstruction -> Close Holes`
- `Filters -> Remeshing, Simplification and Reconstruction -> Simplification: Clustering Decimation`

## Non-homogeneous bodies
For strongly non-homogeneous bodies, there are two problems:
- MeshLab assumes uniform-density bodies. 
- MeshLab computes the Inertia Tensor relative to the computed center of mass. However, for strongly non-homogeneous bodies, the computed center of mass will be far from the real center of mass, and therefore the computed inertia tensor might be just wrong.