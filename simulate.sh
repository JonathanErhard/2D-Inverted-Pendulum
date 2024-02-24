i=1
while [ $i -le 10 ]
do
    ./simulate.out $i
    ((i++))
done