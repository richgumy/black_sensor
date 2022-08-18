function [raw_vm] = get_raw_vm(serial)
    % Function to turn ERT data in serial format into an array or zeros if
    % invalid
    % @param serial - serial handle
    % @output - processed serial measurement data
    raw_vm = str2num(readline(serial));
    if (isempty(raw_vm))
        raw_vm = zeros(1,16);
    end
end

