class TransactionHistoryModel {
  final String title, date, amount;
  final bool isWithdraw;

  const TransactionHistoryModel({
    required this.title,
    required this.date,
    required this.amount,
    required this.isWithdraw,
  });
}
