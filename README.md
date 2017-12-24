# Spatial Reconstruction Server


- Created a main class, ProcessXYZ, that houses all the functions etc. for the whole project
- At some point, we should create header files and move the custom class definitions there.

## TODO
- [ ] Fix a lot of the code
- [ ] Conversion function between point cloud and mesh
- [ ] Outlier removal testing
- [ ] Documentation
- [ ] Implement either ransac or the density thing for plane detection + isolation + corner + edge detection to create outer bounds for the room

## VCG Library
- Created templated classes MyMesh, MyFace, MyEdge, MyVertex and a struct MyUsedTypes
    - All of these extend prebuilt classes so we can use those public functions
- Created a class MyDelaunayFlip. Not quite sure what it does, but is a part of the hole filling algo
- Import export etc. working
- Created a ransac function that doesn’t work
- Picked up the example from `trimesh_ransac.cpp` in `vcglib/apps/sample/trimesh_ransac/`
- Created a hole filling function that doesn’t quite work
- Prolly the face normal issue. See github issue #1
Picked up the example from `trimesh_hole.cpp` in `vcglib/apps/sample/trimesh_hole/`

- [ ] Need a view model function at a later stage

- [ ] Need to complete printVertexLocation

- [ ] Need to complete createBoundingBox

**I think this would be required to follow that density calc method that I talked about**

## PCL Library
- point cloud stuff
- PSD data format
- Can import OBJs
- Can perform statistical outlier removal and radius outlier removal. 
- None seem quite useful so far. 
- [ ] Need to test with varying parameters.
- Some problem with the conditional outlier removal
- Some problem with the view model function
- Most stuff requires pointers to work (i.e. internal functions etc.)
- Can create a pointer or const pointer using makeShared
- See second line of importOBJAsPSD function

