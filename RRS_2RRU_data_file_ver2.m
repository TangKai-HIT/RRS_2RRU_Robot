% Simscape(TM) Multibody(TM) version: 7.6

% This is a model data file derived from a Simscape Multibody Import XML file using the smimport function.
% The data in this file sets the block parameter values in an imported Simscape Multibody model.
% For more information on this file, see the smimport function help page in the Simscape Multibody documentation.
% You can modify numerical values, but avoid any other changes to this file.
% Do not add code to this file. Do not edit the physical units shown in comments.

%%%VariableName:smiData


%============= RigidTransform =============%

%Initialize the RigidTransform structure array by filling in null values.
smiData.RigidTransform(33).translation = [0.0 0.0 0.0];
smiData.RigidTransform(33).angle = 0.0;
smiData.RigidTransform(33).axis = [0.0 0.0 0.0];
smiData.RigidTransform(33).ID = "";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(1).translation = [0 -15.660289284221165 30.003342271758928];  % mm
smiData.RigidTransform(1).angle = 0;  % rad
smiData.RigidTransform(1).axis = [0 0 0];
smiData.RigidTransform(1).ID = "UCS[并联模块2RRU-RRS_ver2_multibodyLink:sm_fixed_frame]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(2).translation = [1.5570922828905211e-08 -15.660289284220623 231.94002796075412];  % mm
smiData.RigidTransform(2).angle = 9.4942909012052128e-11;  % rad
smiData.RigidTransform(2).axis = [-0.70712307802394803 0.70709048373838912 1.8235909473939204e-05];
smiData.RigidTransform(2).ID = "UCS[并联模块2RRU-RRS_ver2_multibodyLink:sm_floating_frame]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(3).translation = [0 -16.187729584119722 -106.63566083123592];  % mm
smiData.RigidTransform(3).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(3).axis = [0 1 0];
smiData.RigidTransform(3).ID = "UCS[并联模块动平台_withMotor_ver2*:*默认:sm_BTC]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(4).translation = [-15.599999999579921 23.881951005929398 -4.2674197509029455e-13];  % mm
smiData.RigidTransform(4).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(4).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(4).ID = "B[虎克铰装配_ver2:虎克铰零件1_ver2-1:-:虎克铰装配_ver2:虎克铰零件2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(5).translation = [-15.600000000000517 8.259536811927786e-09 2.2355598092438766e-08];  % mm
smiData.RigidTransform(5).angle = 2.0943951023931926;  % rad
smiData.RigidTransform(5).axis = [0.57735026918962873 0.57735026918962484 0.57735026918962384];
smiData.RigidTransform(5).ID = "F[虎克铰装配_ver2:虎克铰零件1_ver2-1:-:虎克铰装配_ver2:虎克铰零件2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(6).translation = [-15.59999999957992 23.881951005929405 -4.2500725161431774e-13];  % mm
smiData.RigidTransform(6).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(6).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(6).ID = "B[虎克铰装配_ver2:虎克铰零件1_ver2-2:-:虎克铰装配_ver2:虎克铰零件2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(7).translation = [2.9933226480218457e-09 1.1066380604940949e-08 -15.59999999933936];  % mm
smiData.RigidTransform(7).angle = 1.5237471480510581e-16;  % rad
smiData.RigidTransform(7).axis = [0.97765401242166039 0.2102204366750009 1.5658243087014311e-17];
smiData.RigidTransform(7).ID = "F[虎克铰装配_ver2:虎克铰零件1_ver2-2:-:虎克铰装配_ver2:虎克铰零件2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(8).translation = [-20.802079999999997 16.949234715121538 -99.528090758404332];  % mm
smiData.RigidTransform(8).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(8).axis = [1 0 0];
smiData.RigidTransform(8).ID = "B[并联模块RRU分支_ver2:并联模块RRU-RU杆_ver2-1:-:并联模块RRU分支_ver2:虎克铰装配_ver2-1:虎克铰零件1_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(9).translation = [-20.802079999585516 -2.1180489940681895 16.949234715124856];  % mm
smiData.RigidTransform(9).angle = 1.570796326794897;  % rad
smiData.RigidTransform(9).axis = [-1 -3.3619373058597304e-16 -5.1043853311990626e-16];
smiData.RigidTransform(9).ID = "F[并联模块RRU分支_ver2:并联模块RRU-RU杆_ver2-1:-:并联模块RRU分支_ver2:虎克铰装配_ver2-1:虎克铰零件1_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(10).translation = [-41.600000000000001 0 -109.20531106112678];  % mm
smiData.RigidTransform(10).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(10).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(10).ID = "B[并联模块RRU分支_ver2:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2:并联模块RRU-RU杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(11).translation = [-41.600000000000541 -9.4757979240966961e-09 124.07190925362285];  % mm
smiData.RigidTransform(11).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(11).axis = [0.57735026918962606 0.57735026918962562 0.57735026918962551];
smiData.RigidTransform(11).ID = "F[并联模块RRU分支_ver2:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2:并联模块RRU-RU杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(12).translation = [0 0 0];  % mm
smiData.RigidTransform(12).angle = 0;  % rad
smiData.RigidTransform(12).axis = [0 0 0];
smiData.RigidTransform(12).ID = "B[并联模块RRU分支_ver2:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(13).translation = [0 0 0];  % mm
smiData.RigidTransform(13).angle = 0;  % rad
smiData.RigidTransform(13).axis = [0 0 0];
smiData.RigidTransform(13).ID = "F[并联模块RRU分支_ver2:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(14).translation = [0 0 0];  % mm
smiData.RigidTransform(14).angle = 0;  % rad
smiData.RigidTransform(14).axis = [0 0 0];
smiData.RigidTransform(14).ID = "B[并联模块RRS分支_ver2:并联模块RRS-RR杆_ver2-1:-:并联模块RRS分支_ver2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(15).translation = [0 0 0];  % mm
smiData.RigidTransform(15).angle = 0;  % rad
smiData.RigidTransform(15).axis = [0 0 0];
smiData.RigidTransform(15).ID = "F[并联模块RRS分支_ver2:并联模块RRS-RR杆_ver2-1:-:并联模块RRS分支_ver2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(16).translation = [-41.600000000000001 0 -110.36059057535645];  % mm
smiData.RigidTransform(16).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(16).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(16).ID = "B[并联模块RRS分支_ver2:并联模块RRS-RR杆_ver2-1:-:并联模块RRS分支_ver2:并联模块RRS-RS杆_V2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(17).translation = [-41.600000000000001 1.2789769243681803e-13 129.9167301375002];  % mm
smiData.RigidTransform(17).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(17).axis = [0.57735026918962584 0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(17).ID = "F[并联模块RRS分支_ver2:并联模块RRS-RR杆_ver2-1:-:并联模块RRS分支_ver2:并联模块RRS-RS杆_V2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(18).translation = [-104.00000000000009 5.1397107157788318 30.003342271758928];  % mm
smiData.RigidTransform(18).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(18).axis = [0.57735026918962651 -0.5773502691896244 0.57735026918962651];
smiData.RigidTransform(18).ID = "B[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-1:并联模块RRU-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(19).translation = [-20.799999999999628 2.8421709430404007e-14 98.79468893887325];  % mm
smiData.RigidTransform(19).angle = 2.0943951023931957;  % rad
smiData.RigidTransform(19).axis = [0.57735026918962573 0.57735026918962584 0.57735026918962573];
smiData.RigidTransform(19).ID = "F[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-1:并联模块RRU-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(20).translation = [103.99999999999996 -36.460289284221162 30.003342271758928];  % mm
smiData.RigidTransform(20).angle = 2.0943951023931975;  % rad
smiData.RigidTransform(20).axis = [-0.57735026918962651 -0.57735026918962451 -0.57735026918962629];
smiData.RigidTransform(20).ID = "B[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-2:并联模块RRU-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(21).translation = [-20.799999999999983 3.5527136788005009e-15 98.794688938873492];  % mm
smiData.RigidTransform(21).angle = 2.0943951023932006;  % rad
smiData.RigidTransform(21).axis = [0.57735026918962629 0.57735026918962751 0.57735026918962351];
smiData.RigidTransform(21).ID = "F[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-2:并联模块RRU-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(22).translation = [4.2009451473035142e-10 31.683251005929392 5.8499999999995724];  % mm
smiData.RigidTransform(22).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(22).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(22).ID = "B[并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块动平台_withMotor_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(23).translation = [103.99999999893507 -10.337729605820547 43.965639156811349];  % mm
smiData.RigidTransform(23).angle = 3.1415926535897927;  % rad
smiData.RigidTransform(23).axis = [-0.70710678118654802 -0.70710678118654713 4.7997677230877918e-16];
smiData.RigidTransform(23).ID = "F[并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块动平台_withMotor_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(24).translation = [-5.1467959850545892e-09 87.812270410733419 36.164339168764045];  % mm
smiData.RigidTransform(24).angle = 0;  % rad
smiData.RigidTransform(24).axis = [0 0 0];
smiData.RigidTransform(24).ID = "B[并联模块动平台_withMotor_ver2-2:-:并联模块RRS分支_ver2-1:并联模块RRS-RS杆_V2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(25).translation = [3.2947650632275406e-15 2.8421709430404007e-14 -119.68326986249988];  % mm
smiData.RigidTransform(25).angle = 0;  % rad
smiData.RigidTransform(25).axis = [0 0 0];
smiData.RigidTransform(25).ID = "F[并联模块动平台_withMotor_ver2-2:-:并联模块RRS分支_ver2-1:并联模块RRS-RS杆_V2_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(26).translation = [20.800000000000001 88.339710715779219 30.003342271758928];  % mm
smiData.RigidTransform(26).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(26).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(26).ID = "B[并联模块3R基座_ver2-1:-:并联模块RRS分支_ver2-1:并联模块RRS-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(27).translation = [20.800000000000001 -4.2632564145606011e-14 97.639409424643574];  % mm
smiData.RigidTransform(27).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(27).axis = [-0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(27).ID = "F[并联模块3R基座_ver2-1:-:并联模块RRS分支_ver2-1:并联模块RRS-RR杆_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(28).translation = [4.2009451473035142e-10 31.683251005929392 -5.8500000000004224];  % mm
smiData.RigidTransform(28).angle = 2.0943951023931953;  % rad
smiData.RigidTransform(28).axis = [0.57735026918962584 -0.57735026918962584 0.57735026918962584];
smiData.RigidTransform(28).ID = "B[并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块动平台_withMotor_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(29).translation = [-103.99999999893618 -10.337729605815451 43.965639156805295];  % mm
smiData.RigidTransform(29).angle = 3.1415926535897931;  % rad
smiData.RigidTransform(29).axis = [-0.70710678118654791 0.70710678118654724 -1.8244653088236801e-16];
smiData.RigidTransform(29).ID = "F[并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块动平台_withMotor_ver2-2]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(30).translation = [0 0 0];  % mm
smiData.RigidTransform(30).angle = 0;  % rad
smiData.RigidTransform(30).axis = [0 0 0];
smiData.RigidTransform(30).ID = "B[并联模块3R基座_ver2-1:-:]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(31).translation = [0 0 0];  % mm
smiData.RigidTransform(31).angle = 0;  % rad
smiData.RigidTransform(31).axis = [0 0 0];
smiData.RigidTransform(31).ID = "F[并联模块3R基座_ver2-1:-:]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(32).translation = [-2.9057888762769739 -9.5022491276746397 19.999999999999993];  % mm
smiData.RigidTransform(32).angle = 0;  % rad
smiData.RigidTransform(32).axis = [0 0 0];
smiData.RigidTransform(32).ID = "AssemblyGround[并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件1_ver2-1]";

%Translation Method - Cartesian
%Rotation Method - Arbitrary Axis
smiData.RigidTransform(33).translation = [-2.9057888762769601 -9.5022491276746663 19.999999999999996];  % mm
smiData.RigidTransform(33).angle = 0;  % rad
smiData.RigidTransform(33).axis = [0 0 0];
smiData.RigidTransform(33).ID = "AssemblyGround[并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件1_ver2-1]";


%============= Solid =============%
%Center of Mass (CoM) %Moments of Inertia (MoI) %Product of Inertia (PoI)

%Initialize the Solid structure array by filling in null values.
smiData.Solid(8).mass = 0.0;
smiData.Solid(8).CoM = [0.0 0.0 0.0];
smiData.Solid(8).MoI = [0.0 0.0 0.0];
smiData.Solid(8).PoI = [0.0 0.0 0.0];
smiData.Solid(8).color = [0.0 0.0 0.0];
smiData.Solid(8).opacity = 0.0;
smiData.Solid(8).ID = "";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(1).mass = 2.6597662577807553;  % kg
smiData.Solid(1).CoM = [0 5.8510651763014137 1.4889682077116313];  % mm
smiData.Solid(1).MoI = [8091.9153070034472 17647.250840635515 25205.162296327238];  % kg*mm^2
smiData.Solid(1).PoI = [-47.23988633546675 0 0];  % kg*mm^2
smiData.Solid(1).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(1).opacity = 1;
smiData.Solid(1).ID = "并联模块3R基座_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(2).mass = 0.73998737458612041;  % kg
smiData.Solid(2).CoM = [0 0 0.67415810963352907];  % mm
smiData.Solid(2).MoI = [2776.9559668858228 2781.192397035154 165.58879227142879];  % kg*mm^2
smiData.Solid(2).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(2).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(2).opacity = 1;
smiData.Solid(2).ID = "并联模块RRU-RU杆_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(3).mass = 0.014432261284369361;  % kg
smiData.Solid(3).CoM = [-1.4002772844763672e-09 7.0601633135686397 1.424912308555667e-12];  % mm
smiData.Solid(3).MoI = [1.9232612434949243 2.1076309106905451 2.7799326857153095];  % kg*mm^2
smiData.Solid(3).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(3).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(3).opacity = 1;
smiData.Solid(3).ID = "虎克铰零件1_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(4).mass = 0.0063129060346939127;  % kg
smiData.Solid(4).CoM = [0 0 0];  % mm
smiData.Solid(4).MoI = [0.25534005170103213 0.45942238184421724 0.2553400517010323];  % kg*mm^2
smiData.Solid(4).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(4).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(4).opacity = 1;
smiData.Solid(4).ID = "虎克铰零件2_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(5).mass = 0.64205993084243496;  % kg
smiData.Solid(5).CoM = [0 0 -4.8098971656233616];  % mm
smiData.Solid(5).MoI = [2287.3008896119395 2313.2027548048263 182.73418927453676];  % kg*mm^2
smiData.Solid(5).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(5).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(5).opacity = 1;
smiData.Solid(5).ID = "并联模块RRU-RR杆_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(6).mass = 4.9722703521998755;  % kg
smiData.Solid(6).CoM = [-6.3253121477643573e-09 -10.638246119895854 58.780816306294312];  % mm
smiData.Solid(6).MoI = [29087.24823191663 30752.550590117266 10843.873300749943];  % kg*mm^2
smiData.Solid(6).PoI = [1588.5610774850786 -2.8479444607735348e-06 2.5565288005893095e-06];  % kg*mm^2
smiData.Solid(6).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(6).opacity = 1;
smiData.Solid(6).ID = "并联模块动平台_withMotor_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(7).mass = 0.82056439489743527;  % kg
smiData.Solid(7).CoM = [0 0 -4.7377130902778903];  % mm
smiData.Solid(7).MoI = [3748.9465862710899 3753.1462817800216 178.7885145066968];  % kg*mm^2
smiData.Solid(7).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(7).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(7).opacity = 1;
smiData.Solid(7).ID = "并联模块RRS-RS杆_V2_ver2*:*默认";

%Inertia Type - Custom
%Visual Properties - Simple
smiData.Solid(8).mass = 0.66101163951443509;  % kg
smiData.Solid(8).CoM = [0 0 -3.740193911706998];  % mm
smiData.Solid(8).MoI = [2400.8772412074941 2426.7791064003809 188.03638208964244];  % kg*mm^2
smiData.Solid(8).PoI = [0 0 0];  % kg*mm^2
smiData.Solid(8).color = [0.89803921568627454 0.91764705882352937 0.92941176470588238];
smiData.Solid(8).opacity = 1;
smiData.Solid(8).ID = "并联模块RRS-RR杆_ver2*:*默认";


%============= Joint =============%
%X Revolute Primitive (Rx) %Y Revolute Primitive (Ry) %Z Revolute Primitive (Rz)
%X Prismatic Primitive (Px) %Y Prismatic Primitive (Py) %Z Prismatic Primitive (Pz) %Spherical Primitive (S)
%Constant Velocity Primitive (CV) %Lead Screw Primitive (LS)
%Position Target (Pos)

%Initialize the RevoluteJoint structure array by filling in null values.
smiData.RevoluteJoint(10).Rz.Pos = 0.0;
smiData.RevoluteJoint(10).ID = "";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.RevoluteJoint(1).Rz.Pos = -179.99999999848691;  % deg
smiData.RevoluteJoint(1).ID = "[并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件2_ver2-1]";

smiData.RevoluteJoint(2).Rz.Pos = -36.395713715107746;  % deg
smiData.RevoluteJoint(2).ID = "[并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件1_ver2-2:-:并联模块RRU分支_ver2-2:虎克铰装配_ver2-1:虎克铰零件2_ver2-1]";

smiData.RevoluteJoint(3).Rz.Pos = -128.60428628308128;  % deg
smiData.RevoluteJoint(3).ID = "[并联模块RRU分支_ver2-2:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2-2:并联模块RRU-RU杆_ver2-1]";

smiData.RevoluteJoint(4).Rz.Pos = 179.99999999848689;  % deg
smiData.RevoluteJoint(4).ID = "[并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件1_ver2-1:-:并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件2_ver2-1]";

smiData.RevoluteJoint(5).Rz.Pos = -36.39571372678202;  % deg
smiData.RevoluteJoint(5).ID = "[并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件1_ver2-2:-:并联模块RRU分支_ver2-1:虎克铰装配_ver2-1:虎克铰零件2_ver2-1]";

smiData.RevoluteJoint(6).Rz.Pos = -128.60428627909906;  % deg
smiData.RevoluteJoint(6).ID = "[并联模块RRU分支_ver2-1:并联模块RRU-RR杆_ver2-1:-:并联模块RRU分支_ver2-1:并联模块RRU-RU杆_ver2-1]";

smiData.RevoluteJoint(7).Rz.Pos = 128.60428628030698;  % deg
smiData.RevoluteJoint(7).ID = "[并联模块RRS分支_ver2-1:并联模块RRS-RR杆_ver2-1:-:并联模块RRS分支_ver2-1:并联模块RRS-RS杆_V2_ver2-1]";

smiData.RevoluteJoint(8).Rz.Pos = 165.00000000203494;  % deg
smiData.RevoluteJoint(8).ID = "[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-1:并联模块RRU-RR杆_ver2-1]";

smiData.RevoluteJoint(9).Rz.Pos = 165.00000000203511;  % deg
smiData.RevoluteJoint(9).ID = "[并联模块3R基座_ver2-1:-:并联模块RRU分支_ver2-2:并联模块RRU-RR杆_ver2-1]";

smiData.RevoluteJoint(10).Rz.Pos = -104.99999999999962;  % deg
smiData.RevoluteJoint(10).ID = "[并联模块3R基座_ver2-1:-:并联模块RRS分支_ver2-1:并联模块RRS-RR杆_ver2-1]";


%Initialize the SphericalJoint structure array by filling in null values.
smiData.SphericalJoint(1).S.Pos.Angle = 0.0;
smiData.SphericalJoint(1).S.Pos.Axis = [0.0 0.0 0.0];
smiData.SphericalJoint(1).ID = "";

%This joint has been chosen as a cut joint. Simscape Multibody treats cut joints as algebraic constraints to solve closed kinematic loops. The imported model does not use the state target data for this joint.
smiData.SphericalJoint(1).S.Pos.Angle = 179.99999999826582;  % deg
smiData.SphericalJoint(1).S.Pos.Axis = [-2.9961527900413072e-11 0.45091092740266359 0.89256895282598192];
smiData.SphericalJoint(1).ID = "[并联模块动平台_withMotor_ver2-2:-:并联模块RRS分支_ver2-1:并联模块RRS-RS杆_V2_ver2-1]";

