import paramiko

nao = paramiko.SSHClient()
nao.set_missing_host_key_policy(paramiko.AutoAddPolicy())
nao.connect("192.168.0.100", username="nao", password="n@o4")

# ---------- Sad expression ---------------------------------------------------
# Creating LED group:
group_name = "sad_leds"
group_leds = (r'"[\"FaceLedLeft1\", \"FaceLedLeft2\", '
              r'\"FaceLedLeft3\", \"FaceLedLeft4\", '
              r'\"FaceLedRight1\", \"FaceLedRight2\", '
              r'\"FaceLedRight3\", \"FaceLedRight4\"]"')
command = "qicli call --json ALLeds.createGroup "
command = command + r'"\"' + group_name + r'\"" '
command = command + group_leds
stdin, stdout, stderr = nao.exec_command(command)
stdin.close()
stdout.channel.recv_exit_status()  # wait command to finish

# ---------- Happy expression -------------------------------------------------
group1_name = "happy_leds1"
group_leds = (r'"[\"FaceLedLeft0\", \"FaceLedLeft1\", '
              r'\"FaceLedRight0\", \"FaceLedRight1\"]"')
command = "qicli call --json ALLeds.createGroup "
command = command + r'"\"' + group1_name + r'\"" '
command = command + group_leds
stdin, stdout, stderr = nao.exec_command(command)
stdin.close()
stdout.channel.recv_exit_status()  # wait command to finish

group2_name = "happy_leds2"
group_leds = (r'"[\"FaceLedLeft2\", \"FaceLedLeft3\", '
              r'\"FaceLedRight2\", \"FaceLedRight3\"]"')
command = "qicli call --json ALLeds.createGroup "
command = command + r'"\"' + group2_name + r'\"" '
command = command + group_leds
stdin, stdout, stderr = nao.exec_command(command)
stdin.close()
stdout.channel.recv_exit_status()  # wait command to finish

group3_name = "happy_leds3"
group_leds = (r'"[\"FaceLedLeft4\", \"FaceLedLeft5\", '
              r'\"FaceLedRight4\", \"FaceLedRight5\"]"')
command = "qicli call --json ALLeds.createGroup "
command = command + r'"\"' + group3_name + r'\"" '
command = command + group_leds
stdin, stdout, stderr = nao.exec_command(command)
stdin.close()
stdout.channel.recv_exit_status()  # wait command to finish

group4_name = "happy_leds4"
group_leds = (r'"[\"FaceLedLeft6\", \"FaceLedLeft7\", '
              r'\"FaceLedRight6\", \"FaceLedRight7\"]"')
command = "qicli call --json ALLeds.createGroup "
command = command + r'"\"' + group4_name + r'\"" '
command = command + group_leds
stdin, stdout, stderr = nao.exec_command(command)
stdin.close()
stdout.channel.recv_exit_status()  # wait command to finish
