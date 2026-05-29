
SetFactory("OpenCASCADE");

lc = 0.1;

// domain
Box(1) = {-5,-5,0, 15,10,6};

// cube
Box(2) = {-0.5,-0.5,1, 1,1,1};

// subtract cube from domain
BooleanDifference{ Volume{1}; Delete; }{ Volume{2}; }

// physical groups
Physical Volume("fluid") = {1};
Physical Surface("cube") = {7,8,9,10,11,12};
Physical Surface("ground") = {1};
Physical Surface("farfield") = {2,3,4,5,6};

// mesh resolution
Mesh.CharacteristicLengthMin = lc;
Mesh.CharacteristicLengthMax = lc;
