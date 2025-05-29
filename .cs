using UnityEngine.SceneManagement;
using UnityEditor.SceneManagement;
using Hardware.General;
using UnityEditor;
using UnityEngine;

namespace Assets.Editor.Zarnitza.Hardware
{
    public class MotionPlatformTester : BaseEditorWindow
    {
        private string _pathToVehicle = "Assets/Content/Prefabs/Cars/PlayerCar/Lada_2106/Lada 2106 Variant.prefab";
        private string _previousScenePath;

        private GameObject _vehicleController;
        private MotionPlatform _motionPlatform;

        private Vector3 _tiltAngles; // pitch, roll, yaw в градусах
        private Vector3 _translations; // sway, surge, heave в мм
        private Vector2 _speeds = new(5.0f, 20.0f); // Скорости: градусы/с, мм/с
        private Vector3 _sineAmplitudes; // Амплитуды синусоиды: градусы для вращений, мм для смещений
        private Vector3 _sineFrequencies = Vector3.one; // Частоты синусоиды в Гц
        private Vector3 _sinePhases; // Фазы синусоиды в градусах

        private bool _isInitialized;
        private bool _isInTestScene;

        protected override string HelpMessage => "Инструмент тестирования динамической платформы MOTUS 6DOF.\n" +
                                                  "Спавнит машину, подключается к платформе и отправляет команды на основе углов наклона и смещений.";

        [MenuItem("Tools/Testers/Motion Platform Tester")]
        public static void CreateTester()
        {
            GetWindow<MotionPlatformTester>("MOTUS 6DOF Platform Tester");
        }

        protected override void OnGUI()
        {
            // Отображение сообщения помощи
            EditorGUILayout.HelpBox(HelpMessage, MessageType.Info);

            _pathToVehicle = EditorGUILayout.TextField("Путь к префабу автомобиля", _pathToVehicle);

            if (GUILayout.Button("Загрузить и инициализировать"))
                LoadAndInitialize();

            if (_isInTestScene)
                TestValuesGUI();

            // Отображение статуса подключения
            if(_isInitialized)
                EditorGUILayout.LabelField("Статус подключения", _motionPlatform.IsConnected ? "Подключено" : "Не подключено");
        }

        private void Initialize()
        {
            _isInitialized = true;
            GameObject platformObject = new GameObject("MotionPlatform");
            _motionPlatform = platformObject.AddComponent<MotionPlatform>();
            _motionPlatform.Initialize();
        }

        private void LoadAndInitialize()
        {
            var prefab = AssetDatabase.LoadAssetAtPath<GameObject>(_pathToVehicle);
            if (prefab == null)
            {
                EditorUtility.DisplayDialog("Ошибка", "Префаб не найден, проверьте путь.", "OK");
                return;
            }

            var activeScene = SceneManager.GetActiveScene();
            _previousScenePath = activeScene.path;

            if (!EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo())
                return;

            EditorSceneManager.NewScene(NewSceneSetup.DefaultGameObjects, NewSceneMode.Single);
            _isInTestScene = true;
            _vehicleController = Instantiate(prefab);

            if (!_isInitialized)
                Initialize();
            // Запуск платформы
            _motionPlatform.StartPlatform();
        }

        private void TestValuesGUI()
        {
            EditorGUILayout.BeginHorizontal();

            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField("Углы наклона (градусы)", EditorStyles.boldLabel);
            _tiltAngles = EditorGUILayout.Vector3Field("Pitch, Roll, Yaw", _tiltAngles);
            EditorGUILayout.LabelField("Смещения (мм)", EditorStyles.boldLabel);
            _translations = EditorGUILayout.Vector3Field("Sway, Surge, Heave", _translations);
            EditorGUILayout.LabelField("Скорости", EditorStyles.boldLabel);
            _speeds = EditorGUILayout.Vector2Field("Градусы/с, мм/с", _speeds);
            if (GUILayout.Button("Отправить одношаговую команду"))
            {
                if (_motionPlatform.IsConnected)
                {
                    // Validate all parameters before sending
                    if (!MotionPlatformParameterValidator.ValidateRotation(_tiltAngles.x) ||
                        !MotionPlatformParameterValidator.ValidateRotation(_tiltAngles.y) ||
                        !MotionPlatformParameterValidator.ValidateRotation(_tiltAngles.z) ||
                        !MotionPlatformParameterValidator.ValidateTranslation(_translations.x) ||
                        !MotionPlatformParameterValidator.ValidateTranslation(_translations.y) ||
                        !MotionPlatformParameterValidator.ValidateTranslation(_translations.z) ||
                        !MotionPlatformParameterValidator.ValidateSpeedRotation(_speeds.x) ||
                        !MotionPlatformParameterValidator.ValidateSpeedTranslation(_speeds.y))
                    {
                        EditorUtility.DisplayDialog("Ошибка", "Один или несколько параметров вне допустимого диапазона!", "OK");
                    }
                    else
                    {
                        _motionPlatform.SendSingleStep(
                            _tiltAngles.x, _tiltAngles.y, _tiltAngles.z,
                            _translations.x, _translations.y, _translations.z,
                            _speeds.x, _speeds.y);
                    }
                }
                else
                {
                    EditorUtility.DisplayDialog("Ошибка", "Платформа не подключена.", "OK");
                }
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.BeginVertical();
            EditorGUILayout.LabelField("Параметры синусоиды", EditorStyles.boldLabel);
            _sineAmplitudes = EditorGUILayout.Vector3Field("Амплитуды (град/мм)", _sineAmplitudes);
            _sineFrequencies = EditorGUILayout.Vector3Field("Частоты (Гц)", _sineFrequencies);
            _sinePhases = EditorGUILayout.Vector3Field("Фазы (градусы)", _sinePhases);
            if (GUILayout.Button("Отправить синусоидальную команду"))
            {
                if (_motionPlatform.IsConnected)
                {
                    bool valid = true;
                    // Validate amplitudes, frequencies, and phases for all 6 DOFs
                    float[] amps = { _sineAmplitudes.x, _sineAmplitudes.y, _sineAmplitudes.z, _sineAmplitudes.x, _sineAmplitudes.y, _sineAmplitudes.z };
                    float[] freqs = { _sineFrequencies.x, _sineFrequencies.y, _sineFrequencies.z, _sineFrequencies.x, _sineFrequencies.y, _sineFrequencies.z };
                    float[] phases = { _sinePhases.x, _sinePhases.y, _sinePhases.z, _sinePhases.x, _sinePhases.y, _sinePhases.z };
                    for (int i = 0; i < 6; i++)
                    {
                        if (i < 3)
                        {
                            if (!MotionPlatformParameterValidator.ValidateAmplitudeRotation(amps[i]) ||
                                !MotionPlatformParameterValidator.ValidatePhase(phases[i]))
                                valid = false;
                        }
                        else
                        {
                            if (!MotionPlatformParameterValidator.ValidateAmplitudeTranslation(amps[i]))
                                valid = false;
                        }
                        if (!MotionPlatformParameterValidator.ValidateFrequency(freqs[i]))
                            valid = false;
                    }
                    if (!valid)
                    {
                        EditorUtility.DisplayDialog("Ошибка", "Один или несколько параметров синусоиды вне допустимого диапазона!", "OK");
                    }
                    else
                    {
                        _motionPlatform.SendSineWave(amps, freqs, phases);
                    }
                }
                else
                {
                    EditorUtility.DisplayDialog("Ошибка", "Платформа не подключена.", "OK");
                }
            }
            if (GUILayout.Button("Вернуться в среднее положение"))
            {
                if (_motionPlatform.IsConnected)
                {
                    _motionPlatform.ReturnToMiddle();
                }
                else
                {
                    EditorUtility.DisplayDialog("Ошибка", "Платформа не подключена.", "OK");
                }
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.EndHorizontal();

            // Применение углов и смещений к автомобилю
            if (_vehicleController != null)
            {
                _vehicleController.transform.eulerAngles = _tiltAngles;
                _vehicleController.transform.position = _translations / 1000f; // Перевод мм в метры для Unity
            }

            // Отображение текущего состояния платформы
            EditorGUILayout.LabelField("Текущее состояние платформы", EditorStyles.boldLabel);
            var status = _motionPlatform.CurrentStatus;
            EditorGUILayout.LabelField("Pitch (°)", status.Attitudes is { Length: > 0 } ? status.Attitudes[0].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Roll (°)", status.Attitudes is { Length: > 1 } ? status.Attitudes[1].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Yaw (°)", status.Attitudes is { Length: > 2 } ? status.Attitudes[2].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Sway (мм)", status.Attitudes is { Length: > 3 } ? status.Attitudes[3].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Surge (мм)", status.Attitudes is { Length: > 4 } ? status.Attitudes[4].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Heave (мм)", status.Attitudes is { Length: > 5 } ? status.Attitudes[5].ToString("F2") : "N/A");
            EditorGUILayout.LabelField("Статус", GetStatusDescription(status.DOFStatus));
        }

        private static string GetStatusDescription(byte statusCode)
        {
            return statusCode switch
            {
                0 => "Инициализация поиска дна",
                1 => "Инициализация поиска дна завершена",
                2 => "Подъем от дна к середине",
                3 => "Подъем от дна к середине завершен",
                4 => "Выполнение в реальном времени",
                5 => "Выполнение команды",
                8 => "Выполнение скрипта",
                9 => "Выполнение скрипта завершено",
                12 => "Спуск от середины к дну",
                13 => "Спуск от середины к дну завершен",
                14 => "Возврат в среднее положение",
                32 => "Ручной режим",
                33 => "Аварийная остановка",
                55 => "Инициализация системы завершена",
                _ => "Неизвестный статус"
            };
        }

        private void OnDestroy()
        {
            if (!string.IsNullOrEmpty(_previousScenePath))
            {
                EditorSceneManager.OpenScene(_previousScenePath, OpenSceneMode.Single);
            }
            if (_motionPlatform != null)
            {
                _motionPlatform.StopPlatform();
                DestroyImmediate(_motionPlatform.gameObject);
            }
            _isInTestScene = false;
        }
    }
}