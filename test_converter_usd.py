import omni.kit.urdf.importer as urdf_importer
from pathlib import Path

def import_urdf_to_usd(urdf_path, usd_output_path=None):
    """Импортировать URDF в USD через Omniverse URDF Importer"""
    
    # Если путь не указан, создать рядом с URDF
    if usd_output_path is None:
        urdf_file = Path(urdf_path)
        usd_output_path = urdf_file.parent / f"{urdf_file.stem}.usd"
    
    # Настройки импорта
    import_config = urdf_importer.UrdfImporterConfig()
    import_config.merge_fixed_joints = False
    import_config.fix_base = True
    import_config.make_instanceable = True
    import_config.distance_scale = 1.0
    
    # Выполнить импорт
    success = urdf_importer.import_urdf(
        urdf_path=urdf_path,
        usd_path=usd_output_path,
        config=import_config
    )
    
    if success:
        print(f"URDF успешно импортирован в: {usd_output_path}")
        return usd_output_path
    else:
        print("Ошибка импорта URDF")
        return None

# Использование
urdf_file = "/home/aermakov/github/isaac-robolab/assets/urdf_rc5/Robot.urdf"
usd_file = import_urdf_to_usd(urdf_file)