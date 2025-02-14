import 'package:flutter/material.dart';
import 'package:responsive_dashboard/models/transaction_history_model.dart';
import 'package:responsive_dashboard/utils/app_styles.dart';

class TransactionItem extends StatelessWidget {
  const TransactionItem({
    super.key,
    required this.transactionModel,
    required TransactionHistoryModel transactionHistoryModel,
  });

  final TransactionHistoryModel transactionModel;
  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 0,
      color: const Color(0xFFFAFAFA),
      shape: RoundedRectangleBorder(
        borderRadius: BorderRadius.circular(
          12,
        ),
      ),
      child: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            Column(
              crossAxisAlignment: CrossAxisAlignment.start,
              children: [
                Text(
                  transactionModel.title,
                  style: AppStyles.styleSemiBold16(context),
                ),
                Text(
                  transactionModel.date,
                  style: AppStyles.styleRegular16(context).copyWith(
                    color: const Color(
                      0xFFAAAAAA,
                    ),
                  ),
                )
              ],
            ),
            Text(
              transactionModel.amount,
              style: AppStyles.styleSemiBold20(context).copyWith(
                color: transactionModel.isWithdraw ? Colors.red : Colors.green,
              ),
            )
          ],
        ),
      ),
    );
  }
}
