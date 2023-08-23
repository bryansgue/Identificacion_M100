function plot_six_subplots(vec1, vec2, figure_title, legend_vec1, legend_vec2)
% PLOT_SIX_SUBPLOTS Grafica seis subplots, cada uno con la comparación de los elementos de una fila de dos vectores
%   PLOT_SIX_SUBPLOTS(vec1, vec2, figure_title, legend_vec1, legend_vec2) grafica seis subplots, cada uno con la comparación de los 
%   elementos de una fila de dos vectores. Los datos de entrada son:
%
%   - vec1: matriz de tamaño 6 x m, donde cada columna representa una curva del primer vector en cada subplot.
%   - vec2: matriz de tamaño 6 x m, donde cada columna representa una curva del segundo vector en cada subplot.
%   - figure_title: una cadena de texto con el título principal de la figura.
%   - legend_vec1: una celda de cadenas de texto con las leyendas de las curvas del primer vector en cada subplot.
%   - legend_vec2: una celda de cadenas de texto con las leyendas de las curvas del segundo vector en cada subplot.
%
%   Ejemplo de uso:
%   >> vec1 = randn(6, 12);
%   >> vec2 = randn(6, 12);
%   >> figure_title = 'Seis graficos en subplots';
%   >> legend_vec1 = {'Vector 1.1', 'Vector 1.2', 'Vector 1.3', 'Vector 1.4', 'Vector 1.5', 'Vector 1.6', 'Vector 1.7', 'Vector 1.8', 'Vector 1.9', 'Vector 1.10', 'Vector 1.11', 'Vector 1.12'};
%   >> legend_vec2 = {'Vector 2.1', 'Vector 2.2', 'Vector 2.3', 'Vector 2.4', 'Vector 2.5', 'Vector 2.6', 'Vector 2.7', 'Vector 2.8', 'Vector 2.9', 'Vector 2.10', 'Vector 2.11', 'Vector 2.12'};
%   >> plot_six_subplots(vec1, vec2, figure_title, legend_vec1, legend_vec2);

% Configuramos los colores a utilizar en la gráfica
colors = lines(size(vec1, 2));

% Creamos la figura con seis subplots
figure('Name', figure_title);
suptitle(figure_title);
for i = 1:6
    subplot(2, 3, i);
    hold on;
    plot(vec1(i,:), 'Color', colors(1,:));
    plot(vec2(i,:), 'Color', colors(2,:));
    hold off;
    xlabel('Muestras');
    ylabel('Amplitud');
    legend(legend_vec1{i}, legend_vec2{i},'Interpreter','latex');
end
end
