
#!/bin/bash


for file in *.bag; do
    [ -f "$file" ] || continue
    echo "$file"
done

