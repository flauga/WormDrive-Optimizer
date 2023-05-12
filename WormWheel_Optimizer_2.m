% Motor Input constants
Motor_Name = "DCX 10L";
NL_Rpm = 21613;
ST_Torque = 1.5;


% Motor Gearbox constants; not working properly
X1 = [1,4,16,64,256,1024,4096];
Y1 = [0.97,0.90,0.80,0.75,0.65,0.60,0.50];

% Motor GearBox Efficiency value Curve Fitting
[xData, yData] = prepareCurveData( X1, Y1 );

% Set up fittype and options.
ft = fittype( 'power1' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.996743366140932 -0.0806530292685218];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );
GBE_pc = fitresult;

% Optimization constants 
Pa = 0.253;
Fc = 0.37;

% Motor Gearbox Optimization variables 
GBRR = optimvar("GBRR","LowerBound",379.17,"UpperBound",379.17);
GB_E = optimvar("GB_E","LowerBound",0.05,"UpperBound",0.95);


% Worm Drive Optimization variables
WDRR = optimvar("WDRR","LowerBound",1,"UpperBound",50);
M = optimvar("M","LowerBound",0.6,"UpperBound",0.9);
T = optimvar("T","LowerBound",10,"UpperBound",40);
Pdw = optimvar("Pdw","LowerBound",7,"UpperBound",14);
Pdww = optimvar("Pdww","LowerBound",8,"UpperBound",25);
NS = optimvar("NS","LowerBound",1,"UpperBound",8);
CD = optimvar("CD","LowerBound",9.2,"UpperBound",9.2);
WD_E = optimvar("WD_E","LowerBound",0.05,"UpperBound",0.95);

% Starting point
initialPoint.GBRR = repmat(1,size(GBRR));
initialPoint.WDRR = repmat(20,size(WDRR));
initialPoint.M = repmat(0.6,size(M));
initialPoint.T = repmat(10,size(T));
initialPoint.Pdw = repmat(7,size(Pdw));
initialPoint.Pdww = repmat(8,size(Pdww));
initialPoint.NS = ones(size(NS));
initialPoint.CD = repmat(9.5,size(CD));
initialPoint.GB_E = repmat(0.4,size(GB_E));
initialPoint.WD_E = repmat(0.4,size(WD_E));

% Problem definition
problem = optimproblem("ObjectiveSense","Maximize");


% Objective functions
GB_E_R = GB_E * GBRR;
WD_E_R = WD_E * WDRR;
Torque_Output = ST_Torque*(GB_E * GBRR)*(WD_E * WDRR);


% Objective setting
problem.Objective = ( Torque_Output );

% Constraint setting (Not sure if this is exhaustive)
problem.Constraints.constraint1 = (Pdw+Pdww)/2 == CD;
problem.Constraints.constraint2 = (Pdww/T) == M;
problem.Constraints.constraint3 = (T/NS) == WDRR;

problem.Constraints.constraint4 = (((M * NS ) / Pdw )*(cos(Pa)-(Fc*(M * NS ) / Pdw )))/(((cos(Pa)*(M * NS ) / Pdw ))+Fc) == WD_E;
problem.Constraints.constraint5 = (NL_Rpm/(GBRR*WDRR)) >= 15;
problem.Constraints.constraint6 = (GBE_pc.a*GBRR^GBE_pc.b) == GB_E;



%{
I don't know how to make Teeth ,Starts and Module discrete

problem.Constaints.constraint4 = (rem(T,1)) == 0;
problem.Constaints.constraint5 = (rem(M,0.5)) == 0;
problem.Constaints.constraint6 = (rem(NS,1)) == 0;
%}

% Options
options = optimoptions("fmincon","Display","iter","PlotFcn","optimplotfval");
% Display problem information show(problem);

% Solve 
[solution,objectiveValue,reasonSolverStopped] = solve(problem,initialPoint);

% Results
solution
reasonSolverStopped
objectiveValue

% Output Messages
message = sprintf('The max Torque Output is %f', objectiveValue);
uiwait(helpdlg(message));

% Clear variables. Please don't remove, I dont understand variables in MATLAB 
clearvars