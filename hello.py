bin_str = "10010110"
temp_int = int(bin_str[1:], base=2)

temp_str_2 = '0'*(len(bin_str)-1)
temp_str_2 = bin_str[0]+ temp_str_2
temp_int_2 = int(temp_str_2, base=2)

output = -temp_int_2+temp_int

print(output)

x = 5
out = bin(x)[2:].zfill(8)
print(out)

