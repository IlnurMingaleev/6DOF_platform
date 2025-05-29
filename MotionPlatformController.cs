using System;
using System.Net;
using System.Net.Sockets;
using System.Runtime.InteropServices;
using System.Collections;
using System.Linq;
using UnityEngine;

namespace Hardware.General
{
    // Структура для отправки команд на контроллер (SCommand_Typedef)
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct SCommand
    {
        public byte ID; // Фиксированное значение 55
        public byte Cmd; // Команда состояния
        public byte SubCmd; // Подкоманда состояния
        public byte FileNum; // Номер файла скрипта
        public byte CyChoose; // Выбор цилиндра
        public byte DO; // Цифровой выход
        public short JogSpeed; // Скорость в ручном режиме
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] DOFs; // Данные позы (pitch, roll, yaw, sway, surge, heave)
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Amp; // Амплитуда синусоиды
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Fre; // Частота синусоиды
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Pha; // Фаза синусоиды
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Pos; // Целевая позиция для одношагового выполнения
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Spd; // Скорость для одношагового выполнения
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] Rev1; // Зарезервировано
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] Rev2; // Зарезервировано
        public uint Time; // Метка времени
    }

    // Структура для получения состояния от контроллера (NetSend_Typedef)
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct NetSend
    {
        public byte ID; // Фиксированное значение 55
        public byte DOFStatus; // Текущее состояние системы
        public byte DI; // Цифровой вход
        public byte Rev1; // Зарезервировано
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Attitudes; // Текущая поза
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] ErrorCode; // Коды ошибок
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] MotorCode; // Значения кодов моторов
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 6)]
        public float[] Tor; // Крутящий момент моторов
        public uint Version; // Версия прошивки
        public uint Time; // Метка времени
    }

    public class MotionPlatform : MonoBehaviour
    {
        // Ограничения движения (предполагаемые, так как не указаны в документации)
        private const float MaxRotationDegrees = 15.0f; // ±30° для pitch, roll, yaw
        private const float MaxTranslationMm = 160.0f; // ±100 мм для sway, surge, heave
        private const float MaxSpeedDegreesPerSec = 30.0f; // 10°/с
        private const float MaxSpeedMmPerSec = 300.0f; // 50 мм/с
        private const float MaxFrequencyHz = 5.0f; // 5 Гц для синусоиды
        private const float MaxAmplitudeDegrees = 20.0f; // 20° для синусоиды
        private const float MaxAmplitudeMm = 80.0f; // 80 мм для синусоиды
        // Настройки сети
        [SerializeField] private string localIP = "127.0.0.1";
        [SerializeField] private int localPort = 10000;
        [SerializeField] private string controllerIP = "127.0.0.1";
        [SerializeField] private int controllerPort = 5000;
        private const int CommunicationFrequencyHz = 100; // 100 Гц
        private const float TimeoutSeconds = 1.0f; // Таймаут 1 секунда


        private UdpClient _udpClient;
        private IPEndPoint _controllerEndPoint;
        private IPEndPoint _localEndPoint;
        private bool _isRunning;

        // Текущее состояние платформы
        public NetSend CurrentStatus { get; private set; }
        public bool IsConnected { get; private set; }

        public void Initialize()
        {
            _udpClient = new UdpClient();
            _localEndPoint = new IPEndPoint(IPAddress.Parse(localIP), localPort);
            _controllerEndPoint = new IPEndPoint(IPAddress.Parse(controllerIP), controllerPort);
            CurrentStatus = new NetSend();
            IsConnected = false;
        }

        public void StartPlatform()
        {
            if (_isRunning)
                return;

            try
            {
                _udpClient.Client.Bind(_localEndPoint);
                Debug.Log($"UDP клиент запущен и привязан к {localIP}:{localPort}");
                _isRunning = true;
                IsConnected = true;

                // Запуск получения состояния
                StartCoroutine(ReceiveStatusCoroutine());

                // Отправка команды инициализации
                StartCoroutine(SendCommandCoroutine(new SCommand
                {
                    ID = 55,
                    Cmd = 0, // Инициализация системы
                    DOFs = new float[6],
                    Amp = new float[6],
                    Fre = new float[6],
                    Pha = new float[6],
                    Pos = new float[6],
                    Spd = new float[6],
                    Rev1 = new float[3],
                    Rev2 = new float[3],
                    Time = (uint)Environment.TickCount
                }));

                Debug.Log("Команда инициализации отправлена.");
            }
            catch (Exception ex)
            {
                Debug.LogError($"Ошибка запуска платформы: {ex.Message}");
                StopPlatform();
            }
        }

        public void StopPlatform()
        {
            if (!_isRunning)
                return;

            _isRunning = false;
            IsConnected = false;
            _udpClient.Close();
            Debug.Log("Платформа остановлена.");
        }

        public void SendSingleStep(float pitchDeg, float rollDeg, float yawDeg, float swayMm, float surgeMm, float heaveMm, float speedDegPerSec, float speedMmPerSec)
        {
            if (!_isRunning)
            {
                Debug.LogWarning("Платформа не запущена.");
                return;
            }

            // Валидация входных параметров
            if (!ValidateRotation(pitchDeg) || !ValidateRotation(rollDeg) || !ValidateRotation(yawDeg) ||
                !ValidateTranslation(swayMm) || !ValidateTranslation(surgeMm) || !ValidateTranslation(heaveMm) ||
                !ValidateSpeedRotation(speedDegPerSec) || !ValidateSpeedTranslation(speedMmPerSec))
            {
                Debug.LogWarning("Недопустимые параметры для одношагового выполнения.");
                return;
            }

            var command = new SCommand
            {
                ID = 55,
                Cmd = 11, // Выполнение команды
                SubCmd = 1, // Одношаговое выполнение
                DOFs = new float[6],
                Amp = new float[6],
                Fre = new float[6],
                Pha = new float[6],
                Pos = new float[] { pitchDeg, rollDeg, yawDeg, swayMm, surgeMm, heaveMm },
                Spd = new float[] { speedDegPerSec, speedDegPerSec, speedDegPerSec, speedMmPerSec, speedMmPerSec, speedMmPerSec },
                Rev1 = new float[3],
                Rev2 = new float[3],
                Time = (uint)Environment.TickCount
            };

            StartCoroutine(SendCommandCoroutine(command));
            Debug.Log($"Одношаговая команда отправлена: Pitch={pitchDeg}°, Roll={rollDeg}°, Yaw={yawDeg}°, Sway={swayMm}мм, Surge={surgeMm}мм, Heave={heaveMm}мм");
        }

        public void SendSineWave(float[] amplitudes, float[] frequencies, float[] phases)
        {
            if (!_isRunning)
            {
                Debug.LogWarning("Платформа не запущена.");
                return;
            }

            // Валидация входных параметров
            for (var i = 0; i < 6; i++)
            {
                switch (i)
                {
                    case < 3 when (!ValidateAmplitudeRotation(amplitudes[i]) || !ValidatePhase(phases[i])):
                        Debug.LogWarning($"Недопустимые параметры вращения для синусоиды на индексе {i}.");
                        return;
                    case >= 3 when !ValidateAmplitudeTranslation(amplitudes[i]):
                        Debug.LogWarning($"Недопустимые параметры смещения для синусоиды на индексе {i}.");
                        return;
                }

                if (ValidateFrequency(frequencies[i]))
                    continue;
                
                Debug.LogWarning($"Недопустимая частота на индексе {i}.");
                return;
            }

            var command = new SCommand
            {
                ID = 55,
                Cmd = 11, // Выполнение команды
                SubCmd = 2, // Синусоидальное выполнение
                DOFs = new float[6],
                Amp = amplitudes, // [pitch, roll, yaw, sway, surge, heave]
                Fre = frequencies,
                Pha = phases,
                Pos = new float[6],
                Spd = new float[6],
                Rev1 = new float[3],
                Rev2 = new float[3],
                Time = (uint)Environment.TickCount
            };

            StartCoroutine(SendCommandCoroutine(command));
            Debug.Log("Команда синусоидального движения отправлена.");
        }

        public void ReturnToMiddle()
        {
            if (!_isRunning)
            {
                Debug.LogWarning("Платформа не запущена.");
                return;
            }

            var command = new SCommand
            {
                ID = 55,
                Cmd = 2, // Возврат в среднее положение
                DOFs = new float[6],
                Amp = new float[6],
                Fre = new float[6],
                Pha = new float[6],
                Pos = new float[6],
                Spd = new float[6]{30,30,30,300,300,300},
                Rev1 = new float[3],
                Rev2 = new float[3],
                Time = (uint)Environment.TickCount
            };

            StartCoroutine(SendCommandCoroutine(command));
            Debug.Log("Команда возврата в среднее положение отправлена.");
        }

        private IEnumerator SendCommandCoroutine(SCommand command)
        {
            try
            {
                var data = StructureToByteArray(command);
                _udpClient.Send(data, data.Length, _controllerEndPoint);
            }
            catch (Exception ex)
            {
                Debug.LogError($"Ошибка отправки команды: {ex.Message}");
            }
            
            yield return new WaitForSeconds(1f / CommunicationFrequencyHz); // Поддержка 100 Гц
        }

        private IEnumerator ReceiveStatusCoroutine()
        {
            float lastReceiveTime = Time.time;
            while (_isRunning)
            {
                try
                {
                    if (_udpClient.Available > 0)
                    {
                        IPEndPoint remoteEP = null;
                        var result = _udpClient.Receive(ref remoteEP);
                        if (result.Length == Marshal.SizeOf<NetSend>())
                        {
                            NetSend status = ByteArrayToStructure<NetSend>(result);
                            if (status.ID == 55)
                            {
                                CurrentStatus = status;
                                lastReceiveTime = Time.time;
                                if (status.ErrorCode.Any(e => e != 0))
                                {
                                    Debug.LogWarning($"Коды ошибок: [{string.Join(", ", status.ErrorCode)}]");
                                }
                            }
                        }
                    }

                    // Проверка таймаута
                    if (Time.time - lastReceiveTime > TimeoutSeconds)
                    {
                        Debug.LogWarning("Таймаут связи с платформой.");
                        IsConnected = false;
                    }
                }
                catch (Exception ex)
                {
                    Debug.LogError($"Ошибка получения состояния: {ex.Message}");
                    IsConnected = false;
                }

                yield return null;
            }
        }

        private static bool ValidateRotation(float value) => Math.Abs(value) <= MaxRotationDegrees;
        private static bool ValidateTranslation(float value) => Math.Abs(value) <= MaxTranslationMm;
        private static bool ValidateSpeedRotation(float value) => Math.Abs(value) <= MaxSpeedDegreesPerSec;
        private static bool ValidateSpeedTranslation(float value) => Math.Abs(value) <= MaxSpeedMmPerSec;
        private static bool ValidateAmplitudeRotation(float value) => Math.Abs(value) <= MaxAmplitudeDegrees;
        private static bool ValidateAmplitudeTranslation(float value) => Math.Abs(value) <= MaxAmplitudeMm;
        private static bool ValidateFrequency(float value) => value >= 0 && value <= MaxFrequencyHz;
        private static bool ValidatePhase(float value) => value >= -360.0f && value <= 360.0f;

        private static byte[] StructureToByteArray<T>(T structure)
        {
            var size = Marshal.SizeOf<T>();
            var arr = new byte[size];
            var ptr = Marshal.AllocHGlobal(size);
            Marshal.StructureToPtr(structure, ptr, true);
            Marshal.Copy(ptr, arr, 0, size);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }

        private static T ByteArrayToStructure<T>(byte[] bytes) where T : struct
        {
            var ptr = Marshal.AllocHGlobal(bytes.Length);
            Marshal.Copy(bytes, 0, ptr, bytes.Length);
            var structure = Marshal.PtrToStructure<T>(ptr);
            Marshal.FreeHGlobal(ptr);
            return structure;
        }

        private void OnDestroy()
        {
            StopPlatform();
        }
    }
}