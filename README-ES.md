[🇪🇸](/README-ES.md "Spanish")

# ros_image_workshop

Taller de ROS para la adquisición, procesamiento y distribución de imágenes

## Descripción

Este taller pretende enseñar a nuevos usuarios como extraer las imágenes desde diferentes fuentes como:

* Web Cams Integradas
* Web Cams USB
* Cámaras de Color/Profundidad como el Kinect
* Cámaras WiFi  (Cámaras IP, [GoPro](https://github.com/TOTON95/ros-gopro-driver))

Además, este taller es diseñado para aplicar procesamiento de imagen y la creación de overlay, características e información para los usuarios.

Finalmente, la distribución de las imágenes es explorada para proveer diferentes etapas de procesamiento para el entorno de ROS.

Este taller ha sido diseñado para ROS Kinetic, pero es espera que funcione en:

- Melodic
- Noetic

La meta de este repositorio es hacer el código lo más simple, organizado, auto-documentado como sea posible.

Diapositivas PDF complementarias son incluidas para mejor explicación del código.

## Instrucciones

*Este taller asume que ya tienes el entorno de ROS instalado en tu sistema, si no es el caso, por favor ve a la [documentación oficial (inglés)](http://wiki.ros.org/ROS/Installation) y sigue las instrucciones. Después vuelve a este repositorio.*



Antes de empezar, tenemos que instalar algunas cosas:

```bash
sudo apt-get update
sudo apt-get install ros-$ROS_DISTRO-usb-cam
```

