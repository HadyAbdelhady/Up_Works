import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/transaction_history_model.dart';
import 'package:responsive_dashboard/views/widgets/transaction_item.dart';

class TransactionHistoryListView extends StatelessWidget {
  const TransactionHistoryListView({super.key});
  static const items = [
    TransactionHistoryModel(
        title: 'Cash Withdrawal',
        date: '13 Apr, 2022 ',
        amount: '\$20,129',
        isWithdraw: true),
    TransactionHistoryModel(
        title: 'Landing Page project',
        date: '13 Apr, 2022',
        amount: '\$2,000',
        isWithdraw: false),
    TransactionHistoryModel(
        title: 'Juni Mobile App project',
        date: '13 Apr, 2022',
        amount: '\$20,129',
        isWithdraw: false),
  ];
  @override
  Widget build(BuildContext context) {
    return Column(
      children: items
          .map((e) => TransactionItem(
                transactionModel: e,
                transactionHistoryModel: e,
              ))
          .toList(),
    );
  }
}
