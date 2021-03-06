# TD-Raymarching-System
![](https://img.shields.io/badge/version-WIP-red)
![](https://img.shields.io/github/last-commit/yeataro/TD-Raymarching-System)
[![license](https://img.shields.io/github/license/yeataro/TD-Raymarching-System)](LICENSE)


A Raymarching-System based on object COMP can be easily integrated with existing [TouchDesigner] rendering processes.
## Work in progress.

![icon](docs/img/Comp.png)

![system](docs/img/useagePW.png)

![render](docs/img/renderPW.png)

## Todo list
### System
- [ ] SDF Create Dialog ( Integration with `TD-OPCreateDialog-Plus` )
- [ ] Group node
- [ ] Affectors node
###  Rendering
- [x] Contvert View-Rec to Bounding Box, Reduce the fragment shader call. ( Optional )
- [ ] UV, texture Node
- [ ] Color blend
- [x] Shadow
- [ ] Volumetric Rendering
### Generator
- [ ] SDF font texture generator
- [ ] mesh to SDF texture generator - 10%
### Documents
- [ ] Github Wiki

## Reference

#### Raymarching-in-TD by exsstas
 https://github.com/exsstas/Raymarching-in-TD

#### distance functions by Inigo Quilez
http://iquilezles.org/www/articles/distfunctions/distfunctions.htm

#### Ray Marching and Signed Distance Functions by Zerø Wind
http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/

#### sdfgen by Armory 3D
https://github.com/armory3d/sdfgen

## Author
#### Yea Chen (yeataro)
https://github.com/yeataro \
<yeataro@gmail.com>

## License 
[MIT license](https://github.com/yeataro/TD-Raymarching-System/blob/master/LICENSE)

[TouchDesigner]: http://www.derivative.ca/