for i in range(len(right)):
    right[i] = round(2.55*right[i]) # 转为整数
    right[i] = hex(right[i]) #
    right[i] = right[i][2:]
    right[i] = right[i].upper()
    i+=1 
right = left[0]+ " " + left[1]+ " " +left[2] + " " + "00"
str ="09 10 03 ED 00 02 04 "
left = str + left
left = crc16Add(left)
left = left.split(' ')
for i in range(len(revolve)):
    left[i] = '0x' + left[i]
    left[i] =left[i].replace(" ", "")
    left[i] =left[i].replace(",", "")
    left[i] = int(left[i], 16)
    # hand[i] = hex(hand[i])
    i+=1
return left