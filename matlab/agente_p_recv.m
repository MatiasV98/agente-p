% =============================================================================
% agente_p_recv.m — Recepción TCP, FFT en tiempo real y grabación de audio
% UTN-FRP | Técnicas Digitales III | 2026
%
% Protocolo: paquetes de 6 bytes [seq uint32 LE][muestra int16 LE]
% Al presionar Ctrl+C o cerrar la figura: guarda WAV y MAT automáticamente.
% =============================================================================

clc; clear; close all;

%% ── Configuración ──────────────────────────────────────────────────────────
ESP32_IP   = '192.168.1.3';   % IP del ESP32 (la muestra el monitor serie al conectarse)
TCP_PORT   = 3333;
Fs         = 16000;
N_DISPLAY  = 4000;
CHUNK_SIZE = 1200;

% Rango fijo del eje Y en dB — ajustá DB_FLOOR si la señal es muy débil
DB_FLOOR = -80;   % dB mínimo visible (ruido de piso)
DB_CEIL  =   0;   % dB máximo (0 dB = pico de referencia)

fprintf('============================================\n');
fprintf('  AGENTE P — Receptor de Audio ESP32\n');
fprintf('  UTN-FRP | Técnicas Digitales III | 2026\n');
fprintf('============================================\n');
fprintf('Conectando a ESP32 en %s:%d ...\n', ESP32_IP, TCP_PORT);

%% ── Conexión TCP ───────────────────────────────────────────────────────────
try
    t = tcpclient(ESP32_IP, TCP_PORT, 'Timeout', 10);
catch ME
    error('No se pudo conectar: %s', ME.message);
end
fprintf('Conectado al ESP32. Fs=%d Hz\n', Fs);
fprintf('Cerrá la figura o Ctrl+C para guardar y salir.\n\n');

%% ── Figura ─────────────────────────────────────────────────────────────────
fig = figure('Name', 'AGENTE P — Espectro FFT', ...
             'NumberTitle', 'off', 'Position', [100 100 950 500]);

ax_fft = axes('Parent', fig);
f_axis = (0 : N_DISPLAY/2 - 1) * (Fs / N_DISPLAY);

h_fft = plot(ax_fft, f_axis, DB_FLOOR * ones(N_DISPLAY/2, 1), ...
             'r-', 'LineWidth', 1.2);

xlabel(ax_fft, 'Frecuencia (Hz)');
ylabel(ax_fft, 'Magnitud (dB)');
title(ax_fft, sprintf('FFT — Audio filtrado ESP32 (Fs=%d Hz)', Fs));
xlim(ax_fft, [0 Fs/2]);
ylim(ax_fft, [DB_FLOOR DB_CEIL]);   % ← rango fijo, no cambia más
grid(ax_fft, 'on');

% Línea del pico de frecuencia
h_peak = xline(ax_fft, 0, '--', 'Color', [1 0.5 0.1], 'LineWidth', 1.2);

% Texto de estadísticas en el gráfico
h_txt = text(ax_fft, Fs/2 * 0.6, DB_FLOOR + 8, '', ...
             'Color', [0.9 0.9 0.9], 'FontSize', 9, ...
             'FontName', 'Courier New', ...
             'BackgroundColor', [0.15 0.15 0.15]);

%% ── Buffers ──────────────────────────────────────────────────────────────
signal_buf  = zeros(1, N_DISPLAY);
all_samples = [];
pending     = uint8([]);
last_seq    = -1;
total_rx    = 0;
total_lost  = 0;

fprintf('%-20s %-10s %-15s %-10s\n', 'Muestras', 'Tiempo(s)', 'Pico(Hz)', 'Perdidos');
fprintf('%s\n', repmat('-', 1, 60));
t_log = tic;

%% ── Loop principal ───────────────────────────────────────────────────────
try
    while ishandle(fig)

        raw_bytes = read(t, CHUNK_SIZE, 'uint8');
        if isempty(raw_bytes), continue; end

        all_bytes = [pending, raw_bytes(:)'];
        n_usable  = floor(numel(all_bytes) / 6) * 6;
        usable    = all_bytes(1:n_usable);
        pending   = all_bytes(n_usable+1:end);

        n_pkts = n_usable / 6;
        if n_pkts == 0, continue; end

        pkts = reshape(usable, 6, n_pkts)';

        seq = double(pkts(:,1)) + double(pkts(:,2))*256 + ...
              double(pkts(:,3))*65536 + double(pkts(:,4))*16777216;

        raw_u16 = double(pkts(:,5)) + double(pkts(:,6))*256;
        samples = raw_u16;
        samples(samples > 32767) = samples(samples > 32767) - 65536;

        if last_seq >= 0
            gap = seq(1) - last_seq - 1;
            if gap > 0, total_lost = total_lost + gap; end
        end
        if n_pkts > 1
            d = diff(seq);
            total_lost = total_lost + sum(d(d > 1) - 1);
        end
        last_seq = seq(end);

        total_rx    = total_rx + n_pkts;
        all_samples = [all_samples; samples]; %#ok<AGROW>

        if n_pkts >= N_DISPLAY
            signal_buf = samples(end-N_DISPLAY+1:end)';
        else
            signal_buf = [signal_buf(n_pkts+1:end), samples'];
        end

        % ── FFT en dB con rango fijo ──────────────────────────────────
        sig = signal_buf(:) - mean(signal_buf);
        L   = numel(sig);
        w   = 0.5 * (1 - cos(2*pi*(0:L-1)'/(L-1)));
        Y   = fft(sig .* w);
        mag = (2/L) * abs(Y(1:N_DISPLAY/2));

        % Normalizar respecto al pico histórico → dB relativos
        % El pico más alto que haya llegado = 0 dB, todo lo demás negativo
        mag_db = 20 * log10(mag / 32768.0 + 1e-10);
        mag_db = max(mag_db, DB_FLOOR);   % clamp al piso

        set(h_fft, 'YData', mag_db);
        % ylim NO se toca — queda fijo en [DB_FLOOR DB_CEIL]

        % Pico de frecuencia
        [pk_val, pk_idx] = max(mag_db);
        f_peak = f_axis(pk_idx);
        set(h_peak, 'Value', f_peak);

        % Texto de estadísticas
        if toc(t_log) >= 1.0
            fprintf('%-20d %-10.1f %-15.1f %-10d\n', ...
                    total_rx, total_rx/Fs, f_peak, total_lost);
            t_log = tic;
        end

        set(h_txt, 'String', sprintf('Pico: %.0f Hz  (%.1f dB)\nRMS: %.0f\nPerdidos: %d', ...
            f_peak, pk_val, rms(signal_buf), total_lost));

        drawnow limitrate;
    end

catch ME
    if ~strcmp(ME.identifier, 'MATLAB:class:InvalidHandle') && ...
       ~strcmp(ME.identifier, 'MATLAB:Interrupt')
        fprintf('Error: %s\n', ME.message);
    end
end

%% ── Guardar ──────────────────────────────────────────────────────────────
fprintf('\nGuardando archivos...\n');
clear t;

if ~isempty(all_samples)
    % Normalizar por el pico real → usa todo el rango del WAV
pico = max(abs(double(all_samples)));
if pico > 0
    buf_norm = double(all_samples) / pico * 0.95;  % 0.95 para no saturar
else
    buf_norm = double(all_samples) / 32768.0;
end

    audiowrite('audio_filtrado.wav', buf_norm, Fs);
    fprintf('audio_filtrado.wav  (%d muestras, %.2f seg)\n', ...
            numel(all_samples), numel(all_samples)/Fs);

    audio_filtrado = all_samples;
    audio_fs       = Fs;
    perdidos       = total_lost;
    total          = total_rx;
    save('audio_datos.mat', 'audio_filtrado', 'audio_fs', 'perdidos', 'total');
    fprintf('audio_datos.mat\n');

    fprintf('\n── Resumen ────────────────────────────────\n');
    fprintf('  Duración  : %.2f s\n',   numel(all_samples)/Fs);
    fprintf('  Muestras  : %d\n',       total_rx);
    fprintf('  Perdidos  : %d (%.2f%%)\n', total_lost, ...
            100*total_lost/max(total_rx+total_lost,1));
    fprintf('  RMS       : %.1f\n',     rms(double(all_samples)));
    fprintf('────────────────────────────────────────────\n');
else
    fprintf('No se recibieron muestras.\n');
end