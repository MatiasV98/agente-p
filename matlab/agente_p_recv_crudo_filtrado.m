% =============================================================================
% agente_p_recv.m — Recepción TCP, FFT tiempo real, grabación crudo+filtrado
% UTN-FRP | Técnicas Digitales III | 2026
%
% Protocolo paquete 10 bytes:
%   [0-3] seq     uint32 LE
%   [4-5] filtrado int16 LE
%   [6-7] crudo    int16 LE
%   [8-9] padding  0x0000
%
% Al terminar guarda:
%   audio_filtrado.wav / audio_crudo.wav / audio_datos.mat / filtro_info.mat
% =============================================================================

clc; clear; close all;

%% ── Configuración ──────────────────────────────────────────────────────────
ESP32_IP   = '192.168.1.3';   % IP del ESP32 (la muestra el monitor serie al conectarse)
TCP_PORT   = 3333;
Fs         = 16000;
N_DISPLAY  = 4000;
CHUNK_SIZE = 2000;
DB_FLOOR   = -80;
DB_CEIL    =   0;
PKT_SIZE   = 10;

fprintf('============================================\n');
fprintf('  AGENTE P — Receptor de Audio ESP32\n');
fprintf('  UTN-FRP | Técnicas Digitales III | 2026\n');
fprintf('  Protocolo: %d bytes/paquete (seq+filt+crudo)\n', PKT_SIZE);
fprintf('============================================\n');
fprintf('Conectando a ESP32 en %s:%d ...\n', ESP32_IP, TCP_PORT);

%% ── Conexión TCP ───────────────────────────────────────────────────────────
try
    t = tcpclient(ESP32_IP, TCP_PORT, 'Timeout', 10);
catch ME
    error('No se pudo conectar: %s', ME.message);
end
fprintf('Conectado. Fs=%d Hz\n', Fs);
fprintf('Cerrá la figura o Ctrl+C para guardar y salir.\n\n');

%% ── Obtener parámetros del filtro vía HTTP ──────────────────────────────────
filtro_info = struct();
try
    resp_txt = webread(sprintf('http://%s/filtro_info', ESP32_IP));
    filtro_info = jsondecode(resp_txt);
    if isfield(filtro_info, 'tipo')
        fprintf('Filtro activo: %s %s orden %d | fp=%.0fHz fr=%.0fHz | %s\n', ...
            filtro_info.tipo, filtro_info.familia, filtro_info.orden, ...
            filtro_info.fp, filtro_info.fr, filtro_info.tipo_respuesta);
    else
        fprintf('Sin filtro configurado aún.\n');
    end
catch
    fprintf('No se pudo obtener filtro_info (normal si aún no hay filtro).\n');
end

%% ── Figura — dos paneles FFT con eje X logarítmico ─────────────────────────
fig = figure('Name', 'AGENTE P — FFT Crudo vs Filtrado', ...
             'NumberTitle', 'off', 'Position', [50 50 1200 550]);

f_axis = (0 : N_DISPLAY/2 - 1) * (Fs / N_DISPLAY);
f_axis(1) = 1;   % evitar log(0) — el bin DC lo ponemos en 1 Hz

% Panel izquierdo — señal cruda
ax_raw = subplot(1,2,1);
h_raw = plot(ax_raw, f_axis, DB_FLOOR*ones(N_DISPLAY/2,1), 'b-', 'LineWidth', 1.1);
set(ax_raw, 'XScale', 'log');
set(ax_raw, 'XTick', [20 50 100 200 500 1000 2000 5000 8000]);
set(ax_raw, 'XTickLabel', {'20','50','100','200','500','1k','2k','5k','8k'});
xlim(ax_raw, [20 Fs/2]);
ylim(ax_raw, [DB_FLOOR DB_CEIL]);
xlabel(ax_raw, 'Frecuencia (Hz)');
ylabel(ax_raw, 'Magnitud (dB)');
title(ax_raw, 'FFT — Señal CRUDA');
grid(ax_raw, 'on');

% Panel derecho — señal filtrada
ax_flt = subplot(1,2,2);
h_flt = plot(ax_flt, f_axis, DB_FLOOR*ones(N_DISPLAY/2,1), 'r-', 'LineWidth', 1.1);
set(ax_flt, 'XScale', 'log');
set(ax_flt, 'XTick', [20 50 100 200 500 1000 2000 5000 8000]);
set(ax_flt, 'XTickLabel', {'20','50','100','200','500','1k','2k','5k','8k'});
xlim(ax_flt, [20 Fs/2]);
ylim(ax_flt, [DB_FLOOR DB_CEIL]);
xlabel(ax_flt, 'Frecuencia (Hz)');
ylabel(ax_flt, 'Magnitud (dB)');
title(ax_flt, 'FFT — Señal FILTRADA');
grid(ax_flt, 'on');

% Líneas de pico
h_peak_raw = xline(ax_raw, 100, '--', 'Color',[0.2 0.6 1.0], 'LineWidth',1.2);
h_peak_flt = xline(ax_flt, 100, '--', 'Color',[1.0 0.4 0.2], 'LineWidth',1.2);

% Textos de estadísticas
h_txt_raw = text(ax_raw, 3000, DB_FLOOR+8, '', 'Color',[0.9 0.9 0.9], ...
    'FontSize',8, 'FontName','Courier New', 'BackgroundColor',[0.15 0.15 0.15]);
h_txt_flt = text(ax_flt, 3000, DB_FLOOR+8, '', 'Color',[0.9 0.9 0.9], ...
    'FontSize',8, 'FontName','Courier New', 'BackgroundColor',[0.15 0.15 0.15]);

sgtitle(fig, sprintf('AGENTE P — Fs=%dHz | Espectro logarítmico: crudo vs filtrado', Fs), ...
        'FontSize', 11);

%% ── Buffers ──────────────────────────────────────────────────────────────
buf_raw  = zeros(1, N_DISPLAY);
buf_flt  = zeros(1, N_DISPLAY);
all_raw  = [];
all_flt  = [];
pending  = uint8([]);
last_seq = -1;
total_rx = 0;
total_lost = 0;

fprintf('%-18s %-10s %-14s %-14s %-10s\n', ...
        'Muestras','Tiempo(s)','Pico crudo(Hz)','Pico filt(Hz)','Perdidos');
fprintf('%s\n', repmat('-',1,70));
t_log = tic;

%% ── Loop principal ───────────────────────────────────────────────────────
try
    while ishandle(fig)

        raw_bytes = read(t, CHUNK_SIZE, 'uint8');
        if isempty(raw_bytes), continue; end

        all_bytes = [pending, raw_bytes(:)'];
        n_usable  = floor(numel(all_bytes) / PKT_SIZE) * PKT_SIZE;
        usable    = all_bytes(1:n_usable);
        pending   = all_bytes(n_usable+1:end);

        n_pkts = n_usable / PKT_SIZE;
        if n_pkts == 0, continue; end

        pkts = reshape(usable, PKT_SIZE, n_pkts)';

        % Decodificar secuencia uint32 LE
        seq = double(pkts(:,1)) + double(pkts(:,2))*256 + ...
              double(pkts(:,3))*65536 + double(pkts(:,4))*16777216;

        % Decodificar filtrado int16 LE con signo
        flt_u16  = double(pkts(:,5)) + double(pkts(:,6))*256;
        samp_flt = flt_u16;
        samp_flt(samp_flt > 32767) = samp_flt(samp_flt > 32767) - 65536;

        % Decodificar crudo int16 LE con signo
        raw_u16  = double(pkts(:,7)) + double(pkts(:,8))*256;
        samp_raw = raw_u16;
        samp_raw(samp_raw > 32767) = samp_raw(samp_raw > 32767) - 65536;

        % Detección de pérdidas
        if last_seq >= 0
            gap = seq(1) - last_seq - 1;
            if gap > 0, total_lost = total_lost + gap; end
        end
        if n_pkts > 1
            d = diff(seq);
            total_lost = total_lost + sum(d(d > 1) - 1);
        end
        last_seq = seq(end);

        total_rx = total_rx + n_pkts;
        all_raw  = [all_raw;  samp_raw]; %#ok<AGROW>
        all_flt  = [all_flt;  samp_flt]; %#ok<AGROW>

        % Actualizar buffers circulares
        if n_pkts >= N_DISPLAY
            buf_raw = samp_raw(end-N_DISPLAY+1:end)';
            buf_flt = samp_flt(end-N_DISPLAY+1:end)';
        else
            buf_raw = [buf_raw(n_pkts+1:end), samp_raw'];
            buf_flt = [buf_flt(n_pkts+1:end), samp_flt'];
        end

        % ── FFT con ventana Hanning ───────────────────────────────────
        L = N_DISPLAY;
        w = 0.5 * (1 - cos(2*pi*(0:L-1)'/(L-1)));

        sig_r = buf_raw(:) - mean(buf_raw);
        mag_r = (2/L) * abs(fft(sig_r .* w));
        db_r  = max(20*log10(mag_r(1:N_DISPLAY/2)/32768.0 + 1e-10), DB_FLOOR);

        sig_f = buf_flt(:) - mean(buf_flt);
        mag_f = (2/L) * abs(fft(sig_f .* w));
        db_f  = max(20*log10(mag_f(1:N_DISPLAY/2)/32768.0 + 1e-10), DB_FLOOR);

        set(h_raw, 'XData', f_axis, 'YData', db_r);
        set(h_flt, 'XData', f_axis, 'YData', db_f);

        [pk_r_val, pk_r_idx] = max(db_r);
        [pk_f_val, pk_f_idx] = max(db_f);
        f_pk_r = f_axis(pk_r_idx);
        f_pk_f = f_axis(pk_f_idx);

        set(h_peak_raw, 'Value', max(f_pk_r, 20));
        set(h_peak_flt, 'Value', max(f_pk_f, 20));

        set(h_txt_raw, 'String', sprintf('Pico: %.0fHz (%.1fdB)\nRMS: %.0f', ...
            f_pk_r, pk_r_val, rms(buf_raw)));
        set(h_txt_flt, 'String', sprintf('Pico: %.0fHz (%.1fdB)\nRMS: %.0f', ...
            f_pk_f, pk_f_val, rms(buf_flt)));

        if toc(t_log) >= 1.0
            fprintf('%-18d %-10.1f %-14.1f %-14.1f %-10d\n', ...
                    total_rx, total_rx/Fs, f_pk_r, f_pk_f, total_lost);
            t_log = tic;
        end

        drawnow limitrate;
    end

catch ME
    if ~strcmp(ME.identifier,'MATLAB:class:InvalidHandle') && ...
       ~strcmp(ME.identifier,'MATLAB:Interrupt')
        fprintf('Error: %s\n', ME.message);
    end
end

%% ── Obtener filtro_info al final si no lo teníamos ───────────────────────
if ~isfield(filtro_info,'tipo')
    try
        resp_txt    = webread(sprintf('http://%s/filtro_info', ESP32_IP));
        filtro_info = jsondecode(resp_txt);
    catch
        filtro_info = struct('nota','No disponible');
    end
end

%% ── Guardar ──────────────────────────────────────────────────────────────
fprintf('\nGuardando archivos...\n');
clear t;

if ~isempty(all_flt)

    pico = max(max(abs(double(all_raw))), max(abs(double(all_flt))));
    if pico == 0, pico = 32768; end
    factor = 0.95 / pico;

    audiowrite('audio_filtrado.wav', double(all_flt)*factor, Fs);
    fprintf('audio_filtrado.wav  (%d muestras, %.2fs)\n', numel(all_flt), numel(all_flt)/Fs);

    audiowrite('audio_crudo.wav', double(all_raw)*factor, Fs);
    fprintf('audio_crudo.wav     (%d muestras, %.2fs)\n', numel(all_raw), numel(all_raw)/Fs);

    audio_filtrado = all_flt;
    audio_crudo    = all_raw;
    audio_fs       = Fs;
    perdidos       = total_lost;
    total          = total_rx;
    save('audio_datos.mat', 'audio_filtrado', 'audio_crudo', ...
         'audio_fs', 'perdidos', 'total');
    fprintf('audio_datos.mat\n');

    save('filtro_info.mat', 'filtro_info');
    fprintf('filtro_info.mat\n');

    if isfield(filtro_info,'tipo')
        fprintf('\n── Filtro guardado ────────────────────────\n');
        fprintf('  Tipo      : %s\n', filtro_info.tipo);
        if isfield(filtro_info,'familia'),        fprintf('  Familia   : %s\n', filtro_info.familia); end
        if isfield(filtro_info,'orden'),          fprintf('  Orden     : %d\n', filtro_info.orden); end
        if isfield(filtro_info,'tipo_respuesta'), fprintf('  Respuesta : %s\n', filtro_info.tipo_respuesta); end
        if isfield(filtro_info,'fp'),             fprintf('  fp        : %.1f Hz\n', filtro_info.fp); end
        if isfield(filtro_info,'fr'),             fprintf('  fr        : %.1f Hz\n', filtro_info.fr); end
        if isfield(filtro_info,'As_dB'),          fprintf('  As        : %.1f dB\n', filtro_info.As_dB); end
        if isfield(filtro_info,'Rp_dB'),          fprintf('  Rp        : %.1f dB\n', filtro_info.Rp_dB); end
        fprintf('────────────────────────────────────────────\n');
    end

    fprintf('\n── Resumen de sesión ───────────────────────\n');
    fprintf('  Duración  : %.2f s\n',   total_rx/Fs);
    fprintf('  Muestras  : %d\n',       total_rx);
    fprintf('  Perdidos  : %d (%.2f%%)\n', total_lost, ...
            100*total_lost/max(total_rx+total_lost,1));
    fprintf('  RMS crudo : %.1f\n',     rms(double(all_raw)));
    fprintf('  RMS filt  : %.1f\n',     rms(double(all_flt)));
    fprintf('────────────────────────────────────────────\n');
else
    fprintf('No se recibieron muestras.\n');
end