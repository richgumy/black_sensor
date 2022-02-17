% print_convert real_complex01b.png 

% % Create reconstruction model + solve
imdl = mk_common_model('f2c',16);

% Use 16bit or 12bit ADC (chooses with serial data indexing)
ADC12 = 0;
ADC16 = 18;
ADC = ADC16;

% Setup serial connection with ERT device
ert_serial = serialport("COM6",115200);
fprintf("Connected to COM6\n",i);

% Obtain vi 
Vm_raw = str2num(readline(ert_serial));

while (Vm_raw(1:2) ~= [1 0] & Vm_raw(1:2) ~= [0 1]) % find start of serial matrix
    Vm_raw = str2num(readline(ert_serial));    
%     fprintf("%d ",Vm_raw(1+ADC:18+ADC));
%     fprintf("\n");
    fprintf("Waiting...\n",i);
end
fprintf("Setting current reference state\n");
fprintf("%d ",Vm_raw(3+ADC:18+ADC));
fprintf("\n");

vi = zeros(256,1);
vi(1:16) = Vm_raw(3+ADC:18+ADC);
for (i = 2:16)
    Vm_raw = str2num(readline(ert_serial));
    fprintf("%d ",Vm_raw(3+ADC:18+ADC));
    fprintf("\n");
    vi((i-1)*16+1:i*16) = Vm_raw(3+ADC:18+ADC);
end
fprintf("Reference set\n");

% Store all solved elements
dut_r_arr = [];
iter = 1;

% Continuously update vi and vh based on serial readings
while(1)

    Vm_raw = str2num(readline(ert_serial));
    
    while (Vm_raw(1:2) ~= [1 0] & Vm_raw(1:2) ~= [0 1]) % find start of serial matrix
        Vm_raw = str2num(readline(ert_serial));
    end
%     fprintf("%d ",Vm_raw(3+ADC:18+ADC));
%     fprintf("\n");
    
    Vm_arr = zeros(256,1);
    Vm_arr(1:16) = Vm_raw(3+ADC:18+ADC);
%     fprintf("data-1,");
    for (i = 2:16)
%         fprintf("%d,",i);
        Vm_raw = str2num(readline(ert_serial));
%         fprintf("%d ",Vm_raw(3+ADC:18+ADC));
%         fprintf("\n");
        Vm_arr((i-1)*16+1:i*16) = Vm_raw(3+ADC:18+ADC);
    end
%     fprintf("\n");
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
    fprintf("solved!\n",i);

    % v_test is in format [vm01 for i, vm12 for i, vm23 for i, ...]
    % v_test2 is in format [vm for i01, vm for i12, vm for i23, ...]
    
    eidors_colourbar(imgr);
%     imgr.calc_colours.ref_level = -10; %  centre of the colour scale
    imgr.calc_colours.clim = 30;  %  max diff from ref_level
    show_fem(imgr);
%     print_convert real_complex01b.png
    

    title 'Real Conductivity Change'
    
    dut_r_arr(:,iter) = imgr.elem_data;
    iter = iter + 1;
    range_cond = max(imgr.elem_data)-min(imgr.elem_data);
    fprintf("Range:%f\n",range_cond);
    
    pause(0.05);
    
%     vi = vh;
%     
%     fprintf("%d bytes in serial buffer\n",ert_serial.NumBytesAvailable);
%     for i = 1:4
%         fprintf("%ds til next measurement begins!\n",i);
%         pause(1);
%     end
    flush(ert_serial); % Clears serial buffer
end

% Call this function before re-running
clear ert_serial

% Replay animation of stored reconstructions:
for i = 1:length(dut_r_arr)-1
    imgr.elem_data = dut_r_arr(:,i);
    show_fem(imgr);
    pause(0.01);
end