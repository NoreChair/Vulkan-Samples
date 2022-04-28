Gen spirv :

```` bash
glslangValidator -g -V100 --target-env vulkan1.0 -o bin/light_grid.comp.spv light_grid.comp

````

Gen reflection :

```` bash
spirv-cross --reflect --version 310 --es  bin/light_grid.comp.spv --output bin/light_grid.comp.ref
````

