# OpenApex

Simulador de carreras open source (GPLv3) con física de primer nivel, modding abierto y servidores libres.
Estado: prototipo temprano (sim-core) Mucho vibecoding con claude por cierto XD.

## Compilar

    cmake --preset debug
    cmake --build --preset debug
    ctest --preset debug

Requiere clang, CMake >= 3.29, Ninja y vcpkg (`VCPKG_ROOT` definido).
