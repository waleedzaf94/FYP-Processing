[ ] Fix Processing 
[ ] Test ReadFile
    [ ] OBJ - 1462 Segmentation fault
    [x] OFF - Polyhedron Vertices: 292317 Faces: 97439
    [x] PLY - Vertices=0, Faces=0, VertCount=292317

[ ] Test Algorithms 
    [x] Advancing Front - Need to fix output
    [x] Point Shape Detection - Need to merge output
    [ ] Surface Mesh Generation

[ ] Test PrintFile 
    [x] PLY - Working with writePlyPointsAndNormals 
    [x] PLY - Working with writePlyFile
    [ ] OBJ
    [ ] OFF

### Move to PolyhedronBuilder
ToPolyhedron
ToPointAndFacetVectors

### Commands

```
{
    "label": "Test Input OFF",
    "type": "shell",
    "command": "cd bin && ./processing -f ../models/335.off -a -o 335_out.off -p ../models",
    "group": "test"
},
{
    "label": "Test Input OBJ",
    "type": "shell",
    "command": "cd bin && ./processing -f ../models/335.obj -a  -o 335_out.obj -p ../models",
    "group": "test"
},
{
    "label": "Test Input PLY",
    "type": "shell",
    "command": "cd bin && ./processing -f ../models/335.ply -a -o 335_out.ply -p ../models",
    "group": "test"
}
```