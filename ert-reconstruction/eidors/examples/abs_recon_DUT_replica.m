%% Absolute Gauss-Newton inverse solver
%
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

% Prog name: abs_recon_DUT_replica.m
% Author: R Ellingham <richie.ellingham@pg.canterbury.ac.nz>
% Created: 16 Aug 2022
% Description: A program that uses the EIDORS library to generate an 
%   ABSOLUTE EIT resistance reconstruction of data generated by a forward 
%   simulation. A DUT (domain under test) is defined to closely simulate a
%   real llife material.
% Notes:
%       1 - Place this program in eidors/examples/ of eidors-vX.XX-ng directory
%       2 - 
% TODO: Search inline 'TODO's
% clc; clear;

%% Pt A - Make ERT virtual circuit
% 1. Define the material DUT characteristic constants:
%   Geometry consts
cyl_thick = 4;
cyl_rad = 50;
%   Electrode constants
num_elecs = 16;
elec_wdth = 1e-3;
elec_hght = cyl_thick;
%   Homogeneous and inclusion conductivity
cond_h=1.0; 
cond_inc=1.0e-5;
%   SNR for electrode measurements
meas_snr = 200;
%   Inclusion size, location and shape TODO...
i_rad = 10; i_locx = 15; i_locy = 15;

% Set colour bar specs
cc.ref_level = cond_h; cc.clim = 0.6; cc.cb_shrink_move = [.3,.8,.00];


% 2. Create model frameworks:
%   2D forward model without inclusion 
extra_h={'noball','-maxh=2'}; % '-maxh' -> Set max mesh size
mdl_h= ng_mk_cyl_models([0 cyl_rad],[num_elecs],[0.5,cyl_thick]);
%   2D forward model with 'cylindrical' inclusion 
extra_i={'ball','solid ball = cylinder(20,20,0;20,20,4;5) and orthobrick(-1,-1,0;1,1,0.05) -maxh=2;'}
mdl_i = ng_mk_cyl_models([0 cyl_rad],[num_elecs],[0.5,cyl_thick],extra_i);


% 3. Add stimulation patterns to models:
stim=mk_stim_patterns(num_elecs,1,[0,1],[0,1]);
%   ^^ [stim, meas_sel] = mk_stim_patterns( n_elec, n_rings, inj, meas, options, amplitude(default 1mA))
mdl_h.stimulation=stim;
mdl_i.stimulation=stim;


% 4. Forward simulation:
%   Create two images
%       homg:
img_h= mk_image(mdl_h,cond_h); 
img_h.calc_colours = cc;
%       inhomg:
ctr = interp_mesh(mdl_i); ctr=(ctr(:,1)-i_locx).^2 + (ctr(:,2)-i_locy).^2; 
img_i = mk_image(mdl_i, 1 + 0.1*(ctr<i_rad^2));
img_i.calc_colours = cc;
%   Fwd solve
v_i = fwd_solve(img_i); 
%   Add noise to simulated voltages
v_i_n = add_noise(meas_snr, v_i);  
v_h = fwd_solve(img_h);


% 5. Plot
% Homog and inhomog electrode voltages
figure(); hold on; plot(v_i.meas); plot(v_h.meas,'r'); hold off; 
title('Electrode voltage sequence readout');legend('Homog','Inhomog');
% 2D model showing inclusion
figure(); show_fem_enhanced(img_i,[0,1]);
title('FEM model of showing DUT anomaly');
% print_convert last_figure_name.png

%% Pt B - Complete absolute ERT reconstruction

% 1. Create model and constant parameters
%   Create generic mdl structure
imdl = mk_common_model('b2c2',32);
%   Default Gauss Newton solver
imdl.solve = @inv_solve_gn; 
imdl.fwd_model = mdl_i;
imdl.reconst_type = 'absolute';
imdl.jacobian_bkgnd.value= cond_h;

% 2. Add tuning parameters
%   Set max number of iterations
imdl.inv_solve_gn.max_iterations = 3;
%   Set regularisation prior
imdl.RtR_prior=@prior_noser;
%   Set hyperparameter -> generally a larger 'hp' filters out peaks
hp = 2e-3;
imdl.hyperparameter.value = hp;

% 3. Run the inverse solver algorithm
img1   = inv_solve(imdl, v_i); img1.calc_colours = cc; % without noise
img_n1 = inv_solve(imdl, v_i_n); img_n1.calc_colours = cc; % with noise

% 4. Plot
%   GN reconstruction with and without noise
figure(); show_fem_enhanced(img1); eidors_colourbar(img1);
title('1 - GN reconstruction without noise')
figure(); show_fem_enhanced(img_n1); eidors_colourbar(img_n1);
title('1 - GN reconstruction with noise')

%% Pt C - Alternative Gauss Newton solver reconstructions
% 1. Changing prior at each iteration
imdl.solve = @inv_solve_abs_GN_prior;
imdl.hyperparameter.value = 2e-3;
% Solve
img2   = inv_solve(imdl, v_i);     img2.calc_colours   = cc;
img_n2 = inv_solve(imdl, v_i_n);  img_n2.calc_colours = cc; % with noise
vr_agn = fwd_solve(img2); vr_agn_n = fwd_solve(img_n2);

% Plot 'Changing Proir GN Reconstruction' with and without noise
figure(); show_fem_enhanced(img2); eidors_colourbar(img2);
title('2 - Changing Proir GN Reconstruction without noise')
figure(); show_fem_enhanced(img_n2); eidors_colourbar(img_n2);
title('2 - Changing Proir GN Reconstruction with noise') 

% 2. Constrained Gauss Newton solver reconstruction
imdl.solve = @inv_solve_gn;
imdl.hyperparameter.value = hp;
% limit conductivity to be greater than 0 with a log parametrization
imdl.inv_solve_gn.elem_working = 'log_conductivity';
% Solve
img3   = inv_solve(imdl, v_i);    img3.calc_colours   = cc;
img_n3 = inv_solve(imdl, v_i_n);  img_n3.calc_colours = cc; % with noise
%   Plot 'Constrained GN Reconstruction' with and without noise
figure(); show_fem_enhanced(img3); eidors_colourbar(img3);
title('3 - Constrained GN Reconstruction without noise')
figure(); show_fem_enhanced(img_n3); eidors_colourbar(img_n3);
title('3 - Constrained GN Reconstruction with noise')

%% Pt D - Comparing different absolute GN reconstructions
% Diff in result between alg 1 and 2
scale12 = 800;
img_diff12 = img1;
img_diff12.elem_data = (img1.elem_data - img2.elem_data)*scale12;
img_diff_n12 = img_n1;
img_diff_n12.elem_data = (img_n1.elem_data - img_n2.elem_data)*scale12;

figure(); show_fem_enhanced(img_diff12); eidors_colourbar(img_diff12);
title('Diff between algorithm 1 & 2 GN Reconstruction without noise')
figure(); show_fem_enhanced(img_diff_n12); eidors_colourbar(img_diff_n12);
title('Diff between algorithm 1 & 2 GN Reconstruction with noise')

% Diff in result between alg 1 and 3
scale13 = 800;
img_diff13 = img1;
img_diff13.elem_data = (img1.elem_data - img3.elem_data)*scale13;
img_diff_n13 = img_n1;
img_diff_n13.elem_data = (img_n1.elem_data - img_n3.elem_data)*scale13;

figure(); show_fem_enhanced(img_diff13); eidors_colourbar(img_diff13);
title('Diff between algorithm 1 & 3 GN Reconstruction without noise')
figure(); show_fem_enhanced(img_diff_n13); eidors_colourbar(img_diff_n13);
title('Diff between algorithm 1 & 3 GN Reconstruction with noise')

% Diff in result between alg 2 and 3
scale12 = 800;
img_diff12 = img1;
img_diff12.elem_data = (img2.elem_data - img3.elem_data)*scale12;
img_diff_n12 = img_n1;
img_diff_n12.elem_data = (img_n2.elem_data - img_n3.elem_data)*scale12;

figure(); show_fem_enhanced(img_diff12); eidors_colourbar(img_diff12);
title('Diff between algorithm 2 & 3 GN Reconstruction without noise')
figure(); show_fem_enhanced(img_diff_n12); eidors_colourbar(img_diff_n12);
title('Diff between algorithm 2 & 3 GN Reconstruction with noise')