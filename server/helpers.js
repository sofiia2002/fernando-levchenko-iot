const process_messages = (
  num_of_devices,
  active_aggregators,
  processed_parameter,
  exclusive_mess_for_parameter_and_id
) => {
  const data = {};
  const par_aggregators = active_aggregators[processed_parameter];
  for (const parameter of par_aggregators) {
    switch (parameter) {
      case "MAX":
        data.max = process_max(exclusive_mess_for_parameter_and_id);
        break;
      case "MIN":
        data.min = process_min(exclusive_mess_for_parameter_and_id);
        break;
      case "AVG":
        data.avg = process_avg(exclusive_mess_for_parameter_and_id);
        break;
      case "COUNT":
        data.count = process_count(exclusive_mess_for_parameter_and_id);
        break;
      case "SUM":
        data.sum = process_sum(exclusive_mess_for_parameter_and_id);
        break;
      case "VAR":
        data.var = process_variation(exclusive_mess_for_parameter_and_id);
        break;
      default:
        break;
    }
  }

  return data;
};

const process_variation = (messages) => {
  const average = process_avg(messages);
  const variation =
    process_sguareSum(messages) / process_count(messages) - average * average;

  return { average, variation };
};

const process_sum = (messages) => {
  return messages
    .map((value) => value.sum)
    .reduce((sum, value) => sum + value, 0);
};

const process_avg = (messages) => {
  return process_sum(messages) / process_count(messages);
};

const process_max = (messages) => {
  const min_values = messages.map((value) => value.max);
  return Math.max(...min_values);
};

const process_min = (messages) => {
  const min_values = messages.map((value) => value.min);
  return Math.min(...min_values);
};

const process_count = (messages) => {
  return messages
    .map((value) => value.count)
    .reduce((sum, value) => sum + value, 0);
};

const process_sguareSum = (messages) => {
  return messages
    .map((value) => value.squareSum)
    .reduce((sum, value) => sum + value, 0);
};

const convert_message = (message) => {
  let i = 0;
  const bytesResult = Buffer.alloc(103);

  const num_of_par = message.length;
  bytesResult[i++] = num_of_par & 0xff; //(rgb >> 8) & 0xFF;
  for (const par_mess of message) {
    const parameter = par_mess.par;
    bytesResult[i++] = parameter & 0xff;

    if (par_mess.agg.includes("MAX")) bytesResult[i++] = 1 & 0xff;
    else bytesResult[i++] = 0 & 0xff;
    if (par_mess.agg.includes("MIN")) bytesResult[i++] = 1 & 0xff;
    else bytesResult[i++] = 0 & 0xff;
    if (par_mess.agg.includes("COUNT")) bytesResult[i++] = 1 & 0xff;
    else bytesResult[i++] = 0 & 0xff;
    if (par_mess.agg.includes("SUM")) bytesResult[i++] = 1 & 0xff;
    else bytesResult[i++] = 0 & 0xff;
    if (par_mess.agg.includes("SQSUM")) bytesResult[i++] = 1 & 0xff;
    else bytesResult[i++] = 0 & 0xff;

    const ep_duration = par_mess.epDur;
    bytesResult[i++] = (ep_duration >> 8) & 0xff;
    bytesResult[i++] = ep_duration & 0xff;

    const condition_chain = par_mess.cond.chain || 0;
    bytesResult[i++] = condition_chain & 0xff;

    const num_of_conditons = par_mess.cond.agg.length;
    bytesResult[i++] = num_of_conditons & 0xff;

    if (num_of_conditons > 0) {
      for (const single_condition of par_mess.cond.agg) {
        const single_cond_par = single_condition.par;
        bytesResult[i++] = single_cond_par & 0xff;

        const single_cond_type =
          single_condition.code === "G"
            ? 0
            : single_condition.code === "GE"
            ? 1
            : single_condition.code === "L"
            ? 2
            : 3;
        bytesResult[i++] = single_cond_type & 0xff;

        const single_cond_val = single_condition.val;
        bytesResult[i++] = (single_cond_val >> 8) & 0xff;
        bytesResult[i++] = single_cond_val & 0xff;
      }
    }
  }
  return bytesResult;
};

module.exports = { process_messages, convert_message };
