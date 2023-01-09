require("dotenv").config();

const express = require("express");
var cors = require("cors");
var axios = require("axios");
const helpers = require("./helpers");

const router = express.Router();
const app = express();
const port = process.env["BACKEND_PORT"];

const num_of_devices = 1;

var active_aggregators = {};
var saved_messages = [];
var messages_to_be_send_to_frontend = [];
var last_idx = -1;
var processed_mess_ids = [-1];

var corsOptions = {
  allowedHeaders: ["Content-Type"],
  origin: "*",
  preflightContinue: true,
};

app.use(cors(corsOptions));
app.use(express.json());
app.use("/", router);

app.listen(port, () => {
  console.log(`Example app listening on port ${port}`);
});

router.get("/get-messages", function (req, res, next) {
  console.log("request");
  res.send(messages_to_be_send_to_frontend);
  last_idx =
    messages_to_be_send_to_frontend.length > 0
      ? messages_to_be_send_to_frontend[
          messages_to_be_send_to_frontend.length - 1
        ].id
      : last_idx;
  messages_to_be_send_to_frontend = [];
});

router.post("/post-uplink-message", async (req, res) => {
  const data = req.body.uplink_message.decoded_payload;
  console.log(data);

  var message = {
    id:
      messages_to_be_send_to_frontend.length > 0
        ? messages_to_be_send_to_frontend[
            messages_to_be_send_to_frontend.length - 1
          ].id + 1
        : last_idx + 1,
    type: "uplink",
    timestamp: req.body.received_at,
    entityId: req.body.end_device_ids.device_id,
    messDesc: "Forward uplink data message",
    dataPreview: {
      payload: "",
      fPort: req.body.uplink_message.f_port,
      dataRate: "SF7BW125",
      snr: req.body.uplink_message.rx_metadata[0].snr,
      rssi: req.body.uplink_message.rx_metadata[0].rssi,
    },
  };

  if (data) {
    var sendData = {};

    message.dataPreview.payload = JSON.stringify(data);
    messages_to_be_send_to_frontend.push(message);

    console.log(data.AggState);

    if (data.measurementId === 0) {
      processed_mess_ids = [-1];
      saved_messages = [];
    }

    if (data.AggState === 1) {
      saved_messages.push(data);

      console.log("processed_mess_ids", JSON.stringify(processed_mess_ids));
      console.log("saved_messages", JSON.stringify(saved_messages));

      if (data.measurementId - 1 > Math.max(...processed_mess_ids)) {
        const exclusive_mess_for_parameter_and_id = saved_messages.filter(
          (mess) => mess.measurementId === Math.max(...processed_mess_ids) + 1
        );
        console.log(
          "exclusive_mess_for_parameter_and_id",
          JSON.stringify(exclusive_mess_for_parameter_and_id)
        );

        if (exclusive_mess_for_parameter_and_id.length > 0) {
          sendData.data = helpers.process_messages(
            num_of_devices,
            active_aggregators,
            exclusive_mess_for_parameter_and_id[0].parameter,
            exclusive_mess_for_parameter_and_id
          );

          sendData.parameter = exclusive_mess_for_parameter_and_id[0].parameter;

          processed_mess_ids.push(
            exclusive_mess_for_parameter_and_id[0].measurementId
          );

          console.log("sendData", JSON.stringify(sendData));

          message.type = "result";
          message.entityId = "Backend application";
          message.messDesc = "Aggregation result";
          message.dataPreview.fPort = null;
          message.dataPreview.dataRate = null;
          message.dataPreview.rssi = null;
          message.dataPreview.snr = null;
          message.dataPreview.payload = JSON.stringify(sendData);
          messages_to_be_send_to_frontend.push(message);
        }
      }
    }
  }
  console.log(messages_to_be_send_to_frontend);
});

router.post("/add-rule", async (req, res) => {
  console.log(req.body);
  var newData = [];

  var message = {
    id:
      messages_to_be_send_to_frontend.length > 0
        ? messages_to_be_send_to_frontend[
            messages_to_be_send_to_frontend.length - 1
          ].id +
          (last_idx > -1 ? last_idx : 0) +
          1
        : last_idx + 1,
    type: "downlink",
    timestamp: new Date().toISOString(),
    entityId: "All devices",
    messDesc: "Send aggregation message",
    dataPreview: {
      payload: JSON.stringify(req.body),
      fPort: null,
      dataRate: null,
      snr: null,
      rssi: null,
    },
  };
  messages_to_be_send_to_frontend.push(message);

  active_aggregators = {};
  saved_messages = [];
  processed_mess_ids = [-1];

  req.body.forEach((element, index) => {
    newData.push({});
    newData[index].par =
      element.parameter === "temperature"
        ? 0
        : element.parameter === "pressure"
        ? 1
        : 2;
    newData[index].cond = {
      agg: element.conditions.agg.map((cond) => {
        return {
          par:
            cond.parameter === "temperature"
              ? 0
              : cond.parameter === "pressure"
              ? 1
              : 2,
          val: parseInt(cond.value),
          code: cond.conditionCode,
        };
      }),
      chain: element.conditions.chain === "AND" ? 1 : 0,
    };
    newData[index].agg = [
      ...new Set(
        element.aggregators
          .map((agg) =>
            agg === "VAR"
              ? ["COUNT", "SQSUM", "SUM"]
              : agg === "VAR"
              ? ["COUNT", "SUM"]
              : agg
          )
          .flat(1)
      ),
    ];
    if (active_aggregators[element.parameter])
      active_aggregators = {
        ...active_aggregators,
        [element.parameter]: [...element.aggregators],
      };
    else active_aggregators[element.parameter] = [...element.aggregators];
    newData[index].epDur = parseInt(element.epochDuration);
  });

  const result = helpers.convert_message(newData);
  console.log(result);

  try {
    await axios.post(
      "http://localhost:1880/send-rule",
      result.toString("base64"),
      {
        headers: {
          "Content-Type": "text/plain",
        },
      }
    );
  } catch (err) {
    console.log(err);
  }
});
