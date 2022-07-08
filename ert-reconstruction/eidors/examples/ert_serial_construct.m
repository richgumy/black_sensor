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
% Notes:  1 - Ensure ReadToTermination included root EIDORS dir
%         2 - Made using Octave
%         3 - Place this program in eidors/examples/
% 
% clc; clear;

isOctave = exist('OCTAVE_VERSION', 'builtin') ~= 0;
% Use matlab equivalent functions and load requirements
if isOctave
    readline = @readline_octv;
    serialport = @serial;
    cputime = @time;
    % Load package:
    pkg load instrument-control
    % Check if serial support exists
    if (exist("serial") ~= 3)
        disp("No Serial Support");
    end
end

% Create reconstruction model + solve
imdl = mk_common_model('f2c',16);

% Setup serial connection with ERT device
comport = "COM4";
%if ert_serial.status == "open"
%  fclose(ert_serial)
%end
ert_serial = serialport(comport,115200); % This should be serialport in MATLAB.
fprintf("Connected to %s \n",comport);

%% 
%% 0 - CREATE INVERSE MODEL
%% 
%clear inv2d;
%inv2d.name= 'EIT inverse';
%%inv2d.solve=       'inv_solve_diff_GN_one_step';
% inv2d.solve=       'np_inv_solve';
%%inv2d.solve=       'aa_inv_total_var';
% inv2d.hyperparameter.value = 3e-3;
%%inv2d.hyperparameter.func = 'select_noise_figure';
%%inv2d.hyperparameter.noise_figure= 2;
%%inv2d.hyperparameter.tgt_elems= 1:4;
%%inv2d.RtR_prior= 'prior_laplace';
% inv2d.R_prior= 'prior_TV';
%%inv2d.RtR_prior= 'prior_gaussian_HPF';
%inv2d.reconst_type= 'difference';
%inv2d.jacobian_bkgnd.value= 1;
%inv2d.fwd_model= mdl_2d_2;
%inv2d.fwd_model.misc.perm_sym= '{y}';
%inv2d= eidors_obj('inv_model', inv2d);

%% 1 - CALIBRATE
% Obtain reference DUT measurements 

flush(ert_serial); % Clears serial buffer -> NOT TRIED IN OCTAVE

Vm_raw = str2num(readline(ert_serial));

while (~isempty(Vm_raw)) % find start of serial matrix
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
%   fprintf("%d ",Vm_raw);
%   fprintf("\n");
  vi((i-1)*16+1:i*16) = Vm_raw
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
    ti = cputime;
    flush(ert_serial);
    Vm_raw = str2num(readline(ert_serial));
    
    % find start of serial matrix
    while (~isempty(Vm_raw)) 
    Vm_raw = str2num(readline(ert_serial))    
    fprintf("Waiting...\n",i);
    end
    
    Vm_raw = get_raw_vm(ert_serial);
    fprintf("%d ",Vm_raw);
    fprintf("\n");
    Vm_arr = zeros(256,1);
    Vm_arr(1:16) = Vm_raw;
    for (i = 2:16)
    Vm_raw = get_raw_vm(ert_serial);
    %    fprintf("%d ",Vm_raw);
    %    fprintf("\n");
    Vm_arr((i-1)*16+1:i*16) = Vm_raw;
    end
    vh = Vm_arr;
    tf = cputime;
    td = tf-ti;
    fprintf("data gathered in %f\n", td);

    % 2b - SOLVE & PLOT
    ti = cputime;
    imgr = inv_solve(imdl,-vi, -vh);
    tf = cputime;
    td = tf-ti;
    fprintf("solved in %f\n", td);

    ti = cputime;

    eidors_colourbar(imgr);
    imgr.calc_colours.ref_level= 0;
    imgr.calc_colours.clim = 500;
    show_fem(imgr);
    title 'Real Conductivity Change'

    tf = cputime;
    td = tf-ti;
    fprintf("plot rendered in %f\n", td);
    %  pic_str = "relax_" + cputime + ".png"
    %  print_convert(pic_str);


    dut_r_arr(:,iter) = imgr.elem_data;
    iter = iter + 1;
    range_cond = max(imgr.elem_data)-min(imgr.elem_data);
    fprintf("Range:%f\n",range_cond);

    %   pause(0.001);
end

