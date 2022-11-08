require("dotenv").config();
const express = require("express");
var cors = require("cors");
var axios = require("axios");

const router = express.Router();
const app = express();
const port = process.env["BACKEND_PORT"];

var active_aggregators = [];

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

router.post("/post-uplink-message", async (req, res) => {
  const data = req.body.uplink_message.decoded_payload;
  console.log(data);
  const idx = active_aggregators.indexOf("VAR");
  var sendData = {};
  if (data) {
    if (idx !== -1 && !data.status) {
      if (data.HumDataOn === 1) {
        const hum_avg = data.humidity_sum / data.humidity_count;
        const hum_variation =
          data.humidity_squareSum / data.humidity_count - hum_avg * hum_avg;
        sendData.humidity_average = hum_avg.toFixed(4);
        sendData.humidity_variation = hum_variation.toFixed(4);
      }
      if (data.PressDataOn === 1) {
        const press_avg = data.pressure_sum / data.pressure_count;
        const press_variation =
          data.pressure_squareSum / data.pressure_count - press_avg * press_avg;
        sendData.pressure_average = press_avg.toFixed(4);
        sendData.pressure_variation = press_variation.toFixed(4);
      }
      if (data.TempDataOn === 1) {
        const temp_avg = data.temperature_sum / data.temperature_count;
        const temp_variation =
          data.temperature_squareSum / data.temperature_count -
          temp_avg * temp_avg;
        sendData.temperature_average = temp_avg.toFixed(4);
        sendData.temperature_variation = temp_variation.toFixed(4);
      }

      const s = JSON.stringify(sendData);

      try {
        await axios.post(
          "http://localhost:1880/send-agg-result",
          Buffer.from(s).toString("base64"),
          {
            headers: {
              "Content-Type": "text/plain",
            },
          }
        );
      } catch (err) {
        console.log(err);
      }
    }
  }
});

router.post("/add-rule", async (req, res) => {
  console.log(req.body);
  var newData = [];

  req.body.forEach((element, index) => {
    newData.push({});
    newData[index].par = element.parameter;
    if (element.conditions.agg.length !== 0)
      newData[index].cond = {
        agg: element.conditions.agg.map((cond) => {
          return {
            par: cond.parameter,
            val: cond.value,
            code: cond.conditionCode,
          };
        })[0],
        chain: element.conditions.chain,
      };
    if (element.aggregators.length !== 0)
      newData[index].agg = [
        ...new Set(
          element.aggregators
            .map((agg) => (agg === "VAR" ? ["COUNT", "SQSUM", "SUM"] : agg))
            .flat(1)
        ),
      ];
    active_aggregators = { [element.parameter]: [...element.aggregators] };
    newData[index].epDur = element.epochDuration;
  });

  const s = JSON.stringify(newData);

  try {
    await axios.post(
      "http://localhost:1880/send-rule",
      Buffer.from(s).toString("base64"),
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
