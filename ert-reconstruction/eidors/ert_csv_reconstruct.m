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
% Created: 2022-03-31
% Description: A program that uses the EIDORS library to generate an EIT 
%   reconstruction of data gathered from a CSV file from the ERT Pressure 
%   Sensor V1.1
% Notes:  1 - Ensure ReadToTermination included root EIDORS dir
%         2 - Made using Octave/Matlab
%         3 - Place this program in eidors/examples/
% 

% Create reconstruction model + solve
imdl = mk_common_model('f2c',16);

% Setup CSV read with ERT device data
filename = 'ert_black_sensor_data\steady_state_UoA1.csv';
csv_index = 1;
ert_data_arr = table2array(readtable(filename));

% Testing code:
% Vm_raw =  readline_array(ert_data_arr, csv_index)
% csv_index = csv_index + 1

%% 
%% 0 - CREATE INVERSE MODEL SOLVER
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

Vm_raw = readline_array(ert_data_arr, csv_index)
csv_index = csv_index + 1

while (length(Vm_raw) ~= 0) % find start of serial matrix
    Vm_raw =  readline_array(ert_data_arr, csv_index)
    csv_index = csv_index + 1   
    fprintf("Waiting...\n",i);
end
while (length(Vm_raw) ~= 16) % find start of serial matrix
    Vm_raw =  readline_array(ert_data_arr, csv_index)
    csv_index = csv_index + 1
    fprintf("Waiting still...\n",i);
end
fprintf("Setting current reference state\n");
fprintf("%d ",Vm_raw);
fprintf("\n");

vi = zeros(256,1);
vi(1:16) = Vm_raw;
for (i = 2:16)
    Vm_raw =  readline_array(ert_data_arr, csv_index)
    csv_index = csv_index + 1
    fprintf("%d ",Vm_raw);
    fprintf("\n");
    vi((i-1)*16+1:i*16) = Vm_raw;
end
fprintf("Reference DUT set\n");

%% 2 - MEASURE AND SOLVE
% Store all solved elements
dut_r_arr = [];
load_arr = [];
time_arr = [];
iter = 1;

% Continuously update vi and vh based on serial readings
while(1)
    % 2a - MEASURE
    % Obtain current boundary electrode measurement
    ti = cputime;
    Vm_raw =  readline_array(ert_data_arr, csv_index);
    csv_index = csv_index + 1;
    
    % find start of serial matrix
    while (length(Vm_raw) ~= 0) 
        Vm_raw =  readline_array(ert_data_arr, csv_index);
        csv_index = csv_index + 1  ;  
        fprintf("Waiting...\n",i);
    end
    Vm_raw =  readline_array(ert_data_arr, csv_index);
    csv_index = csv_index + 1;
%     fprintf("%d ",Vm_raw);
%     fprintf("\n");
    Vm_arr = zeros(256,1);
    Vm_arr(1:16) = Vm_raw;
    for (i = 2:16)
        Vm_raw =  readline_array(ert_data_arr, csv_index);
        csv_index = csv_index + 1;
%         fprintf("%d ",Vm_raw);
%         fprintf("\n");
        Vm_arr((i-1)*16+1:i*16) = Vm_raw;
    end
    vh = Vm_arr;
    tf = cputime;
    td1 = tf-ti;
%     fprintf("data gathered in %f\n", td1);

    % 2b - SOLVE & PLOT
    ti = cputime;
    imgr = inv_solve(imdl,vi, vh);
    tf = cputime;
    td2 = tf-ti;
%     fprintf("solved in %f\n", td2);

    ti = cputime;
    
    figure(1);
    eidors_colourbar(imgr);
    imgr.calc_colours.ref_level= 0;
    imgr.calc_colours.clim = 150;
    show_fem(imgr);
    title 'Real Conductivity Change'

    tf = cputime;
    td3 = tf-ti;
%     fprintf("plot rendered in %f\n", td3);
    %  pic_str = "relax_" + cputime + ".png"
    %  print_convert(pic_str);
    

    dut_r_arr(:,iter) = imgr.elem_data;
    iter = iter + 1;
    range_cond = max(imgr.elem_data)-min(imgr.elem_data);
%     fprintf("Range:%f\n",range_cond);
%     
%     fprintf("CSV index: %d\n\n",csv_index);
    
    % Plot force vs time
    load_arr(iter) = str2num(char(ert_data_arr(csv_index-1,2))).';
    time_arr(iter) = str2num(char(ert_data_arr(csv_index-1,1))).';
    fprintf("Load : %3.0fg  @ %.2fs ",load_arr(iter),time_arr(iter));
    figure(2);
    plot(time_arr,load_arr)
    title('Load applied')
    xlabel('time [s]') 
    ylabel('load [g]') 
    
    % real time from CSV data
    if (iter>1)
        time_p = (time_arr(iter)-time_arr(iter-1))/1000;
        time_scale = 2; % e.g. 2 = 2x speed
        scaled_time = (time_p - td1 - td2 - td3)/time_scale;
        if scaled_time < 0
            fprintf("time dilated!\n"); % i.e. 
        end
        pause(scaled_time);
    else
        pause(0.1);
    end
    
end

