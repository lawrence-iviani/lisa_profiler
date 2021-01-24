
#!/bin/bash



for file in *.bag; do
    [ -f "$file" ] || continue
    rosrun lisa_profiler process_bag.py $@ -b "$file"
done
