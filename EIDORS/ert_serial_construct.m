% Copyright (C) 2022 R Ellingham
%
% This program is free software: you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program.  If not, see <https://www.gnu.org/licenses/>.

% Author: R Ellingham <richie.ellingham@pg.canterbury.ac.nz>
% Created: 2022-03-07
% Description: A program that uses the EIDORS library to generate an EIT 
%   reconstruction of data gathered serially from the ERT Pressure Sensor V1.1
% Notes:  1 - Ensure ReadToTermination included root dir.
%         2 - Made using Octave

clc; clear;

isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
% Use matlab equivalent functions
if isOctave
  readline = @readline_octv;
endif

% Load package:
pkg load instrument-control

% Check if serial support exists
if (exist("serial") != 3)
    disp("No Serial Support");
endif 

% Create reconstruction model + solve
imdl = mk_common_model('f2c',16);

% Setup serial connection with ERT device
comport = "COM3";
%if ert_serial.status == "open"
%  fclose(ert_serial)
%end
ert_serial = serial(comport,115200); % This should be serialport in MATLAB.
fprintf("Connected to %s \n",comport);

%% 1 - CALIBRATE
% Obtain reference DUT measurements 
Vm_raw = str2num(readline(ert_serial));

while (length(Vm_raw) ~= 0) % find start of serial matrix
  Vm_raw = str2num(readline(ert_serial));    
  fprintf("Waiting...\n",i);
end
Vm_raw = str2num(readline(ert_serial));
fprintf("Setting current reference state\n");
fprintf("%d ",Vm_raw);
fprintf("\n");

vi = zeros(256,1);
vi(1:16) = Vm_raw;
for (i = 2:16)
  Vm_raw = str2num(readline(ert_serial));
  fprintf("%d ",Vm_raw);
  fprintf("\n");
  vi((i-1)*16+1:i*16) = Vm_raw;
end
fprintf("Reference DUT set\n");

%% 2 - MEASURE AND SOLVE
% Store all solved elements
dut_r_arr = [];
iter = 1;

% Continuously update vi and vh based on serial readings
while(1)
  % 2a - MEASURE
  % Obtain current boundary electrode measurement
  ti = time;
  Vm_raw = str2num(readline(ert_serial));
  while (length(Vm_raw) ~= 0) % find start of serial matrix
    Vm_raw = str2num(readline(ert_serial));    
    fprintf("Waiting...\n",i);
  end
  Vm_raw = str2num(readline(ert_serial));
  Vm_arr = zeros(256,1);
  Vm_arr(1:16) = Vm_raw;
  for (i = 2:16)
    Vm_raw = str2num(readline(ert_serial));
    vi((i-1)*16+1:i*16) = Vm_raw;
  end
  vh = Vm_arr;
  tf = time;
  td = tf-ti;
  fprintf("data gathered in %f\n", td);
  
  %% 2b - SOLVE & PLOT
  ti = time;
  imgr = inv_solve(imdl,-vi, -vh);
  tf = time;
  td = tf-ti;
  fprintf("solved in %f\n", td);
  
  ti = time;
  eidors_colourbar(imgr);
  %     imgr.calc_colours.ref_level = -10; %  centre of the colour scale
  imgr.calc_colours.clim = 30;  %  max diff from ref_level
  show_fem(imgr);
  title 'Real Conductivity Change'
  tf = time;
  td = tf-ti;
  fprintf("plot rendered in %f\n", td);
  %  pic_str = "relax_" + cputime + ".png"
  %  print_convert(pic_str);

  
  dut_r_arr(:,iter) = imgr.elem_data;
  iter = iter + 1
  range_cond = max(imgr.elem_data)-min(imgr.elem_data);
  fprintf("Range:%f\n",range_cond);

  pause(0.01);
end

