function array = getBinaryArray(number)
	bit_number = 4;
	array = zeros(1,bit_number);
	bin_number = dec2bin(number,bit_number);
	for i=1:bit_number
		if(int8(bin_number(i))==49)
			array(i) = 1;
		else
			array(i) = 0;
		endif
	endfor
endfunction

