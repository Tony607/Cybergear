# GPT控制机械臂

## 创建GPT custom action 的 Schema.json文件
创建一个`schema.json`文件保存到此目录，即自定义GPT Action所使用的`schema.json`文件，如下

其中`"https://www.ChangeToYourUrl.com"`需要修改为自己的url，此URL需为公网域名URL，如果您没有公网域名URL可参考视频中的方式获取一个: [GPTs Custom Actions：自定义行为连接真实世界实操指南](https://www.bilibili.com/video/BV1sM411D7bE)
```json

{
  "openapi": "3.1.0",
  "info": {
    "title": "Simple Value Storage API",
    "description": "API for setting and getting a value stored on the server.",
    "version": "v1.0.0"
  },
  "servers": [
    {
      "url": "https://www.ChangeToYourUrl.com",
    }
  ],
  "paths": {
    "/setvalue": {
      "get": {
        "description": "Set a value on the server",
        "operationId": "SetValue",
        "parameters": [
          {
            "name": "value",
            "in": "query",
            "description": "The string value to set",
            "required": true,
            "schema": {
              "type": "string"
            }
          }
        ],
        "responses": {
          "200": {
            "description": "Successful response indicating the value has been set"
          },
          "400": {
            "description": "Bad request when 'value' parameter is missing"
          }
        },
        "deprecated": false
      }
    },
    "/getvalue": {
      "get": {
        "description": "Get the current value stored on the server",
        "operationId": "GetValue",
        "responses": {
          "200": {
            "description": "Successful response with the current value"
          }
        },
        "deprecated": false
      }
    }
  },
  "components": {
    "schemas": {}
  }
}
```
## 启动服务器代码（公网HTTP转发Websocket服务）
**功能介绍**：接收GPT发送的HTTP（`/setvalue`）请求值，并将该值通过WebSocket转发给所有已连接的客户端，即本地（无公网域名的）机械臂上位机程序。


1. 服务器中创建一个`SERVER_API_KEY`环境变量，和自定义GPT的Action中设置的**API_KEY**保持一致。
服务器命令行运行：`export SERVER_API_KEY='xxxx'`，其中`xxxx`请替换为您自己的**API_KEY**，

3. 运行
```
cd server
server_main.py
```

## 启动机械臂上位机客户端（Websocket客户端）
1. 客户端中创建一个`SERVER_API_KEY`环境变量，和自定义GPT的Action中设置的**API_KEY**保持一致。

Windows客户端请在环境变量设置中添加`SERVER_API_KEY`变量，Linux客户端请在命令行中添加`export SERVER_API_KEY='xxxx'`，其中`xxxx`请替换为您自己的**API_KEY**。

2. 在`records`目录中创建一系列csv示教轨迹文件，文件名为`positions-xxxx.csv`，其中`xxxx`为文件名, 您可以用根目录下`notebooks/pcan-robotic-4axis-teach.ipynb`的交互式四轴示教notebook创建这些`positions-xxxx.csv`文件，文件内容格式如下：
```c
motor1,motor2,motor3,motor4
-0.03910124361028444,0.7349126420996406,1.0442893110551612,-0.0001907377737087046
0.010490577553978753,2.026207370107576,2.645342183566033,0.5598153658350498
0.6639581902800025,0.6193255512321656,0.9370946822308692,-1.229686427100022
-0.0581750209811549,0.5251010910200655,0.6288624399176008,0.23899443045700686
-0.057030594338902674,-0.018883039597161755,-0.0005722133211261138,0.0001907377737087046
```
3. 机械臂上电，运行客户端代码`main.py`