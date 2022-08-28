## proprietary

The `proprietary/` directory is where all my own code goes. My ultimate goal
is to only have this directory with my projects, though some libraries are
easier to replicate than others so don't hold your breath.

### boilerplate

`boilerplate.h` is the only file you need to include to get a project running. All code
from the `external/` directory gets included in this file. Static library linkage is also
defined here, since the MSVC compiler allows for that.

Warning : This means that you will run into issues if you do not use MSVC. You can still get
the project to run; just manually tell your compiler link the libraries.

### mathematics

Mostly just includes and redefines GLM functions. The goal is to slowly build up my own implementation
as i work, and slowly reduce the usage of GLM until it is gone.

## external

- **Bullet** : physics library with alot of features
- **GLEW** : GL extention wrangler library
- **GLFW** : works with opengl; for opening a window and getting input
- **GLM** : math library with alot of features
- **OpenAL** : audio library
- **stb_image.h** : image loading library
- **stb_image_write.h** : image writing library