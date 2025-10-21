# Описание

Программный интерфейс на Python для управления коллаборативными роботами серии RC через [ядро](https://dev.rozum.com/rozum-emb/09/07/01).

## Стек

- Python

## Установка зависимостей

 Перейти в рабочую директорию проекта

- Для **стандартной** установки выполнить команду:

```bash
pip install -r requirements.txt
```

## Автоматический запуск скрипта робота

- Создайте unit-файл для службы systemd в командной строке:

```bash
sudo nano /etc/systemd/system/start_python_script.service
```

- Добавьте следующий контент в файл:

```ini
[Unit]
Description=Python Script
After=cloop.target
Wants=cloop.target

[Service]
ExecStartPre=/bin/sleep 15
ExecStart=/usr/bin/python3 /path/to/your/script.py
ExecReload=no
Type=simple

[Install]
WantedBy=multi-user.target
```

**где `/path/to/your/` - путь  к вашему исполняющему pyhton-скрипту**

- Перезагрузите конфигурацию `systemd` и включите вашу службу:

```bash
sudo systemctl daemon-reload
sudo systemctl enable start_python_script.service
sudo systemctl start start_python_script.service
```
