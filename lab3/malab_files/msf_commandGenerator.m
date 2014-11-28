function msf_commandGenerator(block)
%msf_commandGenerator generates commanded roll, pitch, yaw angle
%   

  
%%
%% The setup method is used to setup the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.  
%%   
setup(block);
  
%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the S-function block's basic characteristics such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%% 
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

  % Register parameters
  block.NumDialogPrms     = 1;
  block.DialogPrmsTunable = {'Nontunable'};
  
  

  % Register number of ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 3;
 
  
  % Override input port properties
  block.InputPort(1).DatatypeID  = 0; % double
  block.InputPort(1).Complexity  = 'Real';
  block.InputPort(1).Dimensions   = 1;
  block.InputPort(1).SamplingMode = 0;
  block.InputPort(1).DirectFeedthrough = true;
  
  % Override output port properties
  block.OutputPort(1).DatatypeID  = 0; % double
  block.OutputPort(1).Complexity  = 'Real';
  block.OutputPort(1).Dimensions   = 1;
  block.OutputPort(1).SamplingMode = 0;
  
  block.OutputPort(2).DatatypeID  = 0; % double
  block.OutputPort(2).Complexity  = 'Real';
  block.OutputPort(2).Dimensions   = 1;
  block.OutputPort(2).SamplingMode = 0;
  
   
  block.OutputPort(3).DatatypeID  = 0; % double
  block.OutputPort(3).Complexity  = 'Real';
  block.OutputPort(3).Dimensions   = 1;
  block.OutputPort(3).SamplingMode = 0;
 
  % Register sample times
  Td = block.DialogPrm(1).Data;
  block.SampleTimes = [-1 0];
  
  %% -----------------------------------------------------------------
  %% Options
  %% -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back into 
  % M-file
 % block.SetAccelRunOnTLC(false);
  %block.SimStateCompliance = 'DefaultSimState';
  

  %% -----------------------------------------------------------------
  %% Register methods called during update diagram/compilation
  %% -----------------------------------------------------------------
  
  block.RegBlockMethod('CheckParameters', @CheckPrms);
  block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);

  %% -----------------------------------------------------------------
  %% Register methods called at run-time
  %% -----------------------------------------------------------------
  
 block.RegBlockMethod('Outputs', @Outputs);

function CheckPrms(block)
    
function DoPostPropSetup(block)
%% Register all tunable parameters as runtime parameters.
  block.AutoRegRuntimePrms;

%endfunction




function Outputs(block)
  
 t = block.InputPort(1).Data;
 
 yaw = 0;
 pitch = 0;
 roll = 0;
 
 if (t > 100)
     roll = pi/2;
     pitch = 0;
     yaw = 0;
 end
 
 if (t > 200)
     roll = 0;
     pitch = pi/2;
     yaw = 0;
 end

 if (t > 300)
     roll = 0;
     pitch = 0;
     yaw = pi/2;
 end
 

  
 
  block.OutputPort(1).Data = roll;
  block.OutputPort(2).Data = pitch;
  block.OutputPort(3).Data = yaw;

%endfunction


    

