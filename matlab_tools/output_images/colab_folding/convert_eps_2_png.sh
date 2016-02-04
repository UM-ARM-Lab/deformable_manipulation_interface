for file in *.eps;
do
    file=$(basename $file .eps)
    gs -q -sDEVICE=png256 -dEPSCrop -r300 -dNOPAUSE -dSAFER -dBATCH -sOutputFile=$file.png $file.eps
done
