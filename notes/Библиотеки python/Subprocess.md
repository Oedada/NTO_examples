**Библиотека python для взаимодействия с внешними процессами**
все функции этого модуля сводятся к одной модели: создатся отдельный процесс с std:in, std:out и std:err
## Subprocess.run() - запустить и дождаться выполнения

```python
import subprocess

result = subprocess.run(
["ls", "-l"], # команда и аргументы
capture_output=True, # перехват stdout и stderr
text=True, # декодировать в строки (иначе bytes)
check=True, # бросает исключение при коде != 0
cwd="/home/oedada", # рабочая директория
env={"PATH": "/usr/bin"},# окружение
shell=False # запуск через shell
)

type(result)
# >>> subprocess.CompletedProcess

print(result.args)
print(result.returncode)
print(result.stdout)
print(result.stderr)
'''
['ls', '-l']
0
total 80648
drwxr-xr-x 1 oedada users 36 Oct 30 20:19 Arduino
drwxr-xr-x 1 oedada users 654 Oct 24 21:17 Desktop
drwxr-xr-x 1 oedada users 646 Jun 23 22:30 Documents
drwxr-xr-x 1 oedada users 12886 Nov 4 23:56 Downloads
drwxr-xr-x 1 oedada users 214 Dec 30 2024 IdeaProjects
-rw-r--r-- 1 oedada users 82456551 Nov 4 22:31 Lost [Official Music Video] - Linkin Park [7NK_JOkuSVY].mp4
drwxr-xr-x 1 oedada users 50 Oct 27 16:07 Music
.....
'''
```

```python
capture_output=True
# Сокарщение для:
stdout=subprocess.PIPE,
stderr=subprocess.PIPE
```
Т. к. каждый поток можно перенаправить:
- `subprocess.PIPE` — подключить к Python для чтения/записи.
	
- `subprocess.DEVNULL` — перенаправить в `/dev/null`.
    
- `subprocess.STDOUT` — объединить `stderr` со `stdout`.
## Subprocess.Popen - управление процессом
### Основы
```python
from subprocess import Popen, PIPE

proc = Popen(["ping", "-c", "10", "google.com"], stdout=PIPE, text=True)

print(proc.pid) # id процесса
print(f"is finish? {proc.poll()}") # если процесс завершился -> обновляет и возвращает returncode, если нет -> None
# А вот returncode обновляется только привызове poll() или после wait()

i = 0
for line in proc.stdout:
## stdout пополяется когда процесс что-то выводит, до тех пор итератор просто ждёт(простаивает, зависает)
	if i == 3:
		proc.terminate() # или kill() для завершения процесса
	print(">", line.strip())
	i += 1
proc.wait() # жадть завершения процесса и обновить returncode
print("Return code:", proc.returncode)

'''
141655
is finish? None
> PING google.com (142.250.185.142) 56(84) bytes of data.
> 64 bytes from fra16s50-in-f14.1e100.net (142.250.185.142): icmp_seq=1 ttl=64 time=0.073 ms
> 64 bytes from fra16s50-in-f14.1e100.net (142.250.185.142): icmp_seq=2 ttl=64 time=0.112 ms
> 64 bytes from fra16s50-in-f14.1e100.net (142.250.185.142): icmp_seq=3 ttl=64 time=0.138 ms
Return code: -15
'''
```
### Communicate
```python
proc = Popen(["cat"], stdin=PIPE, stdout=PIPE, text=True)
out, err = proc.communicate("Hello World\n") # блокирующий метод общения с процессом

print(out, str(err))
'''
Hello World
 None
'''
```
### Взаимодействие между процессами
```python
p1 = Popen(["ps", "aux"], stdout=PIPE)
p2 = Popen(["grep", "neko"], stdin=p1.stdout, stdout=PIPE, text=True)

output, _ = p2.communicate()
print(output)

'''
oedada 1318 0.0 0.0 7712 5364 ? S 11:41 0:00 /bin/sh /usr/bin/nekoray

oedada 1327 0.3 0.5 1109272 88940 ? Sl 11:41 0:30 /usr/lib/nekoray/nekobox -- -appdata

oedada 1488 0.3 0.3 1260884 52532 ? Sl 11:41 0:32 /usr/lib/nekoray/nekobox_core nekobox -port 42839

oedada 145807 0.0 0.0 6696 2368 pts/3 S+ 14:31 0:00 grep neko
'''
```
### Shell-mode
```python
#Включает синтаксис shell(pipe |, redirection >, &&)
subprocess.run("ls -la | grep py", shell=True)
'''
-rwxr-xr-x 1 oedada users 282 Nov 5 12:36 rofi.sh
-rwxr-xr-x 1 oedada users 2124 Nov 5 14:35 test_rofi.py
'''
```
### Переменные среды
```python
env = os.environ.copy()
env["MYVAR"] = "hello"
print(run(["bash", "-c", "echo $MYVAR"], env=env, capture_output=True, text=True).stdout)
'''
hello
'''
```
### Искючения
```python
try:
	run(["sleep", "10"], timeout=3)
except subprocess.TimeoutExpired:
	print("Процесс не успел завершиться!")
'''
Процесс не успел завершиться!
'''
try:
    subprocess.run(["false"], check=True)
except subprocess.CalledProcessError as e:
    print("Ошибка:", e)
'''
Ошибка: Command '['false']' returned non-zero exit status 1.
'''
```