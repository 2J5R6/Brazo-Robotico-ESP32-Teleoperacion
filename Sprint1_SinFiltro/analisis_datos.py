"""
Script de Análisis de Datos - Sprint 1
Análisis de señales del IMU y cálculo de errores

Uso:
1. Capturar datos del Serial Monitor y guardar en archivo CSV
2. Ejecutar este script para análisis

Requiere: numpy, matplotlib, pandas, scipy
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy import signal
from scipy.fft import fft, fftfreq

# ========== CONFIGURACIÓN ==========
SAMPLE_RATE = 50  # Hz
ACCEL_RANGE = 8  # ±8G
GYRO_RANGE = 500  # ±500°/s

# ========== FUNCIONES DE ANÁLISIS ==========

def load_data(filename):
    """
    Carga datos desde archivo CSV
    Formato esperado: timestamp, accelX, accelY, accelZ, gyroX, gyroY, gyroZ
    """
    try:
        df = pd.read_csv(filename)
        print(f"✓ Datos cargados: {len(df)} muestras")
        return df
    except Exception as e:
        print(f"✗ Error al cargar datos: {e}")
        return None

def calculate_statistics(data):
    """Calcula estadísticas básicas de la señal"""
    stats = {
        'mean': np.mean(data),
        'std': np.std(data),
        'min': np.min(data),
        'max': np.max(data),
        'rms': np.sqrt(np.mean(data**2)),
        'peak_to_peak': np.max(data) - np.min(data)
    }
    return stats

def analyze_noise(data, sample_rate=50):
    """Análisis de ruido en la señal"""
    # FFT para análisis de frecuencias
    N = len(data)
    yf = fft(data)
    xf = fftfreq(N, 1/sample_rate)
    
    # Solo frecuencias positivas
    xf = xf[:N//2]
    yf = 2.0/N * np.abs(yf[:N//2])
    
    # Encontrar frecuencia dominante
    idx_max = np.argmax(yf[1:]) + 1  # Ignorar DC
    dominant_freq = xf[idx_max]
    
    # SNR estimado (simplificado)
    signal_power = np.max(yf)**2
    noise_power = np.mean(yf[1:])**2
    snr_db = 10 * np.log10(signal_power / noise_power) if noise_power > 0 else np.inf
    
    return {
        'dominant_freq': dominant_freq,
        'snr_db': snr_db,
        'freq_spectrum': (xf, yf)
    }

def plot_time_domain(df, axis='accelX'):
    """Gráfica en dominio del tiempo"""
    plt.figure(figsize=(12, 4))
    plt.plot(df['timestamp']/1000, df[axis], linewidth=0.5)
    plt.xlabel('Tiempo (s)')
    plt.ylabel(f'{axis} (m/s² o rad/s)')
    plt.title(f'Señal en el Tiempo - {axis}')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    return plt.gcf()

def plot_frequency_domain(data, sample_rate=50, title='FFT'):
    """Gráfica en dominio de la frecuencia"""
    noise_info = analyze_noise(data, sample_rate)
    xf, yf = noise_info['freq_spectrum']
    
    plt.figure(figsize=(12, 4))
    plt.plot(xf, yf)
    plt.xlabel('Frecuencia (Hz)')
    plt.ylabel('Amplitud')
    plt.title(f'{title} - Frecuencia dominante: {noise_info["dominant_freq"]:.2f} Hz')
    plt.grid(True, alpha=0.3)
    plt.xlim(0, sample_rate/2)
    plt.tight_layout()
    return plt.gcf()

def calculate_position_error(expected, measured):
    """
    Calcula errores entre posiciones esperadas y medidas
    
    Args:
        expected: array de [x, y] posiciones esperadas
        measured: array de [x, y] posiciones medidas
    
    Returns:
        dict con errores calculados
    """
    expected = np.array(expected)
    measured = np.array(measured)
    
    error_x = measured[:, 0] - expected[:, 0]
    error_y = measured[:, 1] - expected[:, 1]
    error_euclidean = np.sqrt(error_x**2 + error_y**2)
    
    return {
        'error_x': error_x,
        'error_y': error_y,
        'error_euclidean': error_euclidean,
        'mean_x': np.mean(np.abs(error_x)),
        'mean_y': np.mean(np.abs(error_y)),
        'mean_euclidean': np.mean(error_euclidean),
        'std_x': np.std(error_x),
        'std_y': np.std(error_y),
        'std_euclidean': np.std(error_euclidean)
    }

def plot_error_analysis(errors_dict):
    """Gráfica de análisis de errores"""
    fig, axes = plt.subplots(1, 3, figsize=(15, 4))
    
    # Error en X
    axes[0].bar(range(len(errors_dict['error_x'])), errors_dict['error_x'])
    axes[0].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[0].set_xlabel('Punto')
    axes[0].set_ylabel('Error X (cm)')
    axes[0].set_title(f'Error en X\nMedia: {errors_dict["mean_x"]:.2f} cm')
    axes[0].grid(True, alpha=0.3)
    
    # Error en Y
    axes[1].bar(range(len(errors_dict['error_y'])), errors_dict['error_y'])
    axes[1].axhline(y=0, color='r', linestyle='--', alpha=0.5)
    axes[1].set_xlabel('Punto')
    axes[1].set_ylabel('Error Y (cm)')
    axes[1].set_title(f'Error en Y\nMedia: {errors_dict["mean_y"]:.2f} cm')
    axes[1].grid(True, alpha=0.3)
    
    # Error Euclidiano
    axes[2].bar(range(len(errors_dict['error_euclidean'])), errors_dict['error_euclidean'])
    axes[2].set_xlabel('Punto')
    axes[2].set_ylabel('Error Euclidiano (cm)')
    axes[2].set_title(f'Error Euclidiano\nMedia: {errors_dict["mean_euclidean"]:.2f} cm')
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_trajectory(expected, measured):
    """Gráfica de trayectoria esperada vs medida"""
    expected = np.array(expected)
    measured = np.array(measured)
    
    plt.figure(figsize=(8, 8))
    plt.plot(expected[:, 0], expected[:, 1], 'go-', label='Esperada', linewidth=2, markersize=10)
    plt.plot(measured[:, 0], measured[:, 1], 'rx-', label='Medida', linewidth=2, markersize=10)
    
    # Líneas de error
    for i in range(len(expected)):
        plt.plot([expected[i, 0], measured[i, 0]], 
                [expected[i, 1], measured[i, 1]], 
                'b--', alpha=0.3)
    
    plt.xlabel('X (cm)')
    plt.ylabel('Y (cm)')
    plt.title('Trayectoria: Esperada vs Medida')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    return plt.gcf()

def generate_report(df, errors_expected_measured=None, errors_measured_estimated=None):
    """Genera reporte completo de análisis"""
    print("\n" + "="*60)
    print("REPORTE DE ANÁLISIS - SPRINT 1")
    print("="*60)
    
    print("\n1. ESTADÍSTICAS DE SEÑALES")
    print("-" * 60)
    
    for axis in ['accelX', 'accelY', 'accelZ']:
        stats = calculate_statistics(df[axis])
        noise = analyze_noise(df[axis], SAMPLE_RATE)
        
        print(f"\n{axis}:")
        print(f"  Media: {stats['mean']:.3f} m/s²")
        print(f"  Desv. Est.: {stats['std']:.3f} m/s²")
        print(f"  Pico-Pico: {stats['peak_to_peak']:.3f} m/s²")
        print(f"  RMS: {stats['rms']:.3f} m/s²")
        print(f"  Freq. dominante: {noise['dominant_freq']:.2f} Hz")
        print(f"  SNR estimado: {noise['snr_db']:.2f} dB")
    
    if errors_expected_measured:
        print("\n2. ERRORES: ESPERADO vs MEDIDO")
        print("-" * 60)
        print(f"  Error medio X: {errors_expected_measured['mean_x']:.2f} ± {errors_expected_measured['std_x']:.2f} cm")
        print(f"  Error medio Y: {errors_expected_measured['mean_y']:.2f} ± {errors_expected_measured['std_y']:.2f} cm")
        print(f"  Error Euclidiano: {errors_expected_measured['mean_euclidean']:.2f} ± {errors_expected_measured['std_euclidean']:.2f} cm")
    
    if errors_measured_estimated:
        print("\n3. ERRORES: MEDIDO vs ESTIMADO")
        print("-" * 60)
        print(f"  Error medio X: {errors_measured_estimated['mean_x']:.2f} ± {errors_measured_estimated['std_x']:.2f} cm")
        print(f"  Error medio Y: {errors_measured_estimated['mean_y']:.2f} ± {errors_measured_estimated['std_y']:.2f} cm")
        print(f"  Error Euclidiano: {errors_measured_estimated['mean_euclidean']:.2f} ± {errors_measured_estimated['std_euclidean']:.2f} cm")
    
    print("\n" + "="*60)

# ========== EJEMPLO DE USO ==========

if __name__ == "__main__":
    print("="*60)
    print("ANÁLISIS DE DATOS - SPRINT 1 (Sin Filtros)")
    print("="*60)
    
    # Ejemplo con datos sintéticos
    print("\n[DEMO] Generando datos sintéticos para demostración...")
    
    # Simular 1000 muestras a 50Hz (20 segundos)
    n_samples = 1000
    t = np.linspace(0, n_samples/SAMPLE_RATE, n_samples)
    
    # Señal sintética con ruido
    signal_clean = 2 * np.sin(2 * np.pi * 1.5 * t)  # 1.5 Hz
    noise = np.random.normal(0, 0.3, n_samples)  # Ruido gaussiano
    signal_noisy = signal_clean + noise
    
    df = pd.DataFrame({
        'timestamp': t * 1000,  # en ms
        'accelX': signal_noisy,
        'accelY': np.random.normal(0, 0.5, n_samples),
        'accelZ': 9.81 + np.random.normal(0, 0.2, n_samples),
        'gyroX': np.random.normal(0, 0.1, n_samples),
        'gyroY': np.random.normal(0, 0.1, n_samples),
        'gyroZ': np.random.normal(0, 0.1, n_samples)
    })
    
    # Análisis de señales
    print("\nGenerando gráficas de análisis...")
    
    # Dominio del tiempo
    fig1 = plot_time_domain(df, 'accelX')
    plt.savefig('analisis_tiempo.png', dpi=150)
    print("✓ Guardado: analisis_tiempo.png")
    
    # Dominio de la frecuencia
    fig2 = plot_frequency_domain(df['accelX'], SAMPLE_RATE, 'Aceleración X')
    plt.savefig('analisis_frecuencia.png', dpi=150)
    print("✓ Guardado: analisis_frecuencia.png")
    
    # Ejemplo de análisis de errores
    print("\nEjemplo de análisis de errores de posición...")
    
    expected_positions = np.array([
        [0, 0],      # Centro
        [10, 0],     # Derecha
        [10, 10],    # Arriba-derecha
        [0, 10],     # Arriba
        [-10, 0]     # Izquierda
    ])
    
    measured_positions = expected_positions + np.random.normal(0, 2, expected_positions.shape)
    
    errors = calculate_position_error(expected_positions, measured_positions)
    
    # Gráfica de errores
    fig3 = plot_error_analysis(errors)
    plt.savefig('analisis_errores.png', dpi=150)
    print("✓ Guardado: analisis_errores.png")
    
    # Gráfica de trayectoria
    fig4 = plot_trajectory(expected_positions, measured_positions)
    plt.savefig('analisis_trayectoria.png', dpi=150)
    print("✓ Guardado: analisis_trayectoria.png")
    
    # Generar reporte
    generate_report(df, errors, None)
    
    print("\n" + "="*60)
    print("Para usar con datos reales:")
    print("1. Capturar datos del Serial Monitor")
    print("2. Guardar en formato CSV")
    print("3. Modificar la sección if __name__ == '__main__'")
    print("4. Cargar datos con load_data('tu_archivo.csv')")
    print("="*60)
    
    # Mostrar gráficas
    # plt.show()  # Descomentar para ver las gráficas
