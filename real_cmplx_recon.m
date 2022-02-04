% imdl = mk_common_model('f2c',16);
% img = mk_image(imdl,1);
% vh = fwd_solve(img);
% 
% DeltaC1 = -0.1;    %Target 1 is non_conductive
% DeltaC2 = 0+0.1i;  %Target 2 has + permittivity
% target= mk_c2f_circ_mapping(img.fwd_model, [[0.5;0.0;0.1],[-0.5;0;0.1]]);
% img.elem_data = 1+ DeltaC1*target(:,1) + DeltaC2*target(:,2) ;
% vi = fwd_solve(img);
% vi = add_noise(5,vi,vh);
% 
% img.calc_colours.component = 'real';
% subplot(321); show_fem(img);
% title 'real conductivity change'
% print_convert real_complex01a.png 
% 
% img.calc_colours.component = 'imag';
% subplot(322); show_fem(img);
% title 'imag conductivity change'
% print_convert real_complex01b.png 
% 
% 
% % Create reconstruction model + solve
imdl = mk_common_model('e2s',16);
% imgr = inv_solve(imdl, vh, vi);
% 
% imgr.calc_colours.clim = 0.02;
% subplot(323); show_fem(imgr);
% title 'real conductivity change'
% print_convert real_complex02a.png 
% 
% imgr.calc_colours.component = 'imag';
% subplot(324); show_fem(imgr);
% title 'imag conductivity change'
% print_convert real_complex02b.png 

% load demo data with complex measurements
% load iirc_data_2006
% vi= v_rotate(:,9); vh= v_reference;

% Use 16bit or 12bit ADC (chooses with serial data indexing)
ADC12 = 0;
ADC16 = 20;
ADC = ADC12;

% Setup serial connection with ERT device
ert_serial = serialport("COM6",115200);
fprintf("Connected to COM6\n",i)

% Obtain vi 
Vm_raw = str2num(readline(ert_serial));

while (Vm_raw(1:2) ~= [0 1]) % find start of serial matrix
    Vm_raw = str2num(readline(ert_serial));
end

vi = zeros(256,1);
vi(1:16) = Vm_raw(3+ADC:18+ADC);
for (i = 2:16)
    Vm_raw = str2num(readline(ert_serial));
    vi((i-1)*16+1:i*16) = Vm_raw(3+ADC:18+ADC);
end

% Continuously update vi and vh based on serial readings
while(1)
    
    Vm_raw = str2num(readline(ert_serial));
    
    while (Vm_raw(1:2) ~= [0 1]) % find start of serial matrix
        Vm_raw = str2num(readline(ert_serial));
    end
    
    Vm_arr = zeros(256,1);
    Vm_arr(1:16) = Vm_raw(3+ADC:18+ADC);
    fprintf("data-1,")
    for (i = 2:16)
        fprintf("%d,",i)
        Vm_raw = str2num(readline(ert_serial));
        Vm_arr((i-1)*16+1:i*16) = Vm_raw(3+ADC:18+ADC);
    end
    fprintf("\n")
    vh = Vm_arr;
    
    % ..................................... %
    % INPUT BEFORE AND AFTER MATRICES HERE: %
    % E.G.  vi = "mat before name"'
    %       vh = "mat after name"'
    % ..................................... %
%     vi = v_MUTv1_i3';
%     vi = vi(:);
%     vh = v_MUTv1_a3'
%     vh = vh(:);
    % ..................................... %

    imgr = inv_solve(imdl, vi, vh);
    fprintf("solved!\n",i)
    % v_test is in format [vm01 for i, vm12 for i, vm23 for i, ...]
    % v_test2 is in format [vm for i01, vm for i12, vm for i23, ...]
    
    
    show_fem(imgr);
%     eidors_colourbar(imgr);
%     img.eidors_colourbar.max_scale = [-40:40];
    title 'real conductivity change'
%     print_convert real_complex03a.png 

%     imgr.calc_colours.component = 'imag';
%     subplot(1); show_fem(imgr);
%     title 'imag conductivity change'
%     print_convert real_complex03b.png
    
    range_cond = max(imgr.elem_data)-min(imgr.elem_data);
    fprintf("Range:%f\n",range_cond)
    
    pause(0.05);
    
%     vi = vh;
    
end

clear ert_serial