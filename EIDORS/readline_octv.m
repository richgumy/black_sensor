## Copyright (C) 2022 richi
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.

## -*- texinfo -*-
## @deftypefn {} {@var{retval} =} readline_octv (@var{input1}, @var{input2})
##
## @seealso{}
## @end deftypefn

function [char_array] = readline_octv (srl_handle, term_char)

    % parameter term_char is optional, if not specified
    % then CR = 'r' = 13dec is the default.
if (nargin == 1)
  term_char = 13;
end

not_terminated = true;
i = 1;
int_array = uint8(1);

while not_terminated

    val = srl_read(srl_handle, 1);

    if(val == term_char)
        not_terminated = false;
    end
        
  % Add char received to array
  int_array(i) = val;
  i = i + 1;

end
    
  % Change int array to a char array and return a string array
  char_array = char(int_array);

endfunction