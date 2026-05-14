
# Sistema de Control Ball and Beam (Bola y Viga)

Este repositorio contiene el desarrollo, modelado e implementación de un sistema **Ball and Beam**. El proyecto se centra en el control de la posición de una esfera que rueda sobre una viga, cuya inclinación es modificada mediante un actuador para mantener el equilibrio en un punto de consigna (setpoint).

##  Descripción del Proyecto
El sistema Ball and Beam es un problema clásico en la teoría de control. Se caracteriza por:
* **Dinámica no lineal:** El movimiento de la bola no tiene una relación lineal simple con el ángulo.
* **Inestabilidad en lazo abierto:** Sin un controlador, la bola caerá de la viga ante cualquier perturbación.
* **Visualización:** Es ideal para observar el desempeño de distintos algoritmos de control en tiempo real.

---

## Estructura del Repositorio

El proyecto está organizado en las siguientes carpetas:

1.  **`01_Informe_Tecnico/`**: Contiene la documentación académica, el modelado matemático detallado (ecuaciones de Euler-Lagrange o Newton) y el análisis de resultados.
2.  **`02_Especificaciones_Tecnicas/`**: Listado de componentes utilizados (sensores ultrasónicos/infrarrojos, servomotores, microcontroladores) y esquemas de conexión.
3.  **`03_Codigos/`**:
    * `Adquisicion_Datos/`: Scripts para la lectura de sensores y filtrado de señales.
    * `Identificacion/`: Código para la obtención de la función de transferencia y parámetros del sistema.
    * `Simulacion/`: Modelos en MATLAB, Simulink o Python para pruebas virtuales.
    * `Controladores/`: Implementación de algoritmos (PID, LQR, Espacio de Estados, etc.).
4.  **`04_Manual_Ejecucion/`**: Guía detallada sobre cómo configurar el entorno, cargar el código en el hardware y calibrar el sistema.
5.  **`05_Datos_Experimentales/`**: Bases de datos y registros (.csv, .xlsx) obtenidos durante las pruebas de laboratorio.
6.  **`06_Fotografias_y_Videos/`**: Registro multimedia del montaje físico y demostraciones del funcionamiento del controlador.

---
##  Instalación y Uso
2.  **Consultar el manual:** Revise la carpeta `04_Manual_Ejecucion/` para instrucciones específicas de puesta en marcha.
3.  **Ejecutar Simulación:** Abra los archivos en `03_Codigos/Simulacion/` para validar el modelo antes de pasar al hardware.

##  Autores
Daniel Felipe Sarmiento Pilonieta -- 2202797
Fabian Andres Amador Ballesteros -- 2204215
David Santiago Arias Avila-2225183 
Elkin Yesid Lozada Cabrera - 2204219 
Jhon Hector Robayo Mayorga - 2225184

---
Este proyecto fue desarrollado con fines educativos para el estudio de sistemas de control dinámicos.
