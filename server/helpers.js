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

module.exports = { process_messages };
