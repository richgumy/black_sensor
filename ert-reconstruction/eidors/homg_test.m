% Creating a resistivity map showing the homogeneity of a piezoresistive
% material.
%  Method:
% 0. Generate FEM model
% 1. Create forward model of homogenous material
% 2. Gather data for inhomogeneity
%   a. Use simulated data
%   b. Use real electrode data
% 3. Create inverse model structure
% 4. Solve inverse model

% 0. Generate FEM model
figure(1)
imdl_2d= mk_common_model('c2c',16);
show_fem(imdl_2d.fwd_model);

% 1. Create forward model of homogenous material
sim_img= mk_image(imdl_2d.fwd_model,1e-8);
% set voltage and current stimulation patterns
stim =  mk_stim_patterns(16,1,[0 1],[0 1],{},0.0001); % 0.1 mA
sim_img.fwd_model.stimulation = stim;
% set homogeneous conductivity and simulate
homg_data=fwd_solve(sim_img);
homg_data.meas

% 2. Gather data for inhomogeneity
%   a. Use simulated data
%   b. Use real electrode data

